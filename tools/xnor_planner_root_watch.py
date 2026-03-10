#!/usr/bin/env python3
import argparse
import csv
import math
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

import numpy as np
import cereal.messaging as messaging
from opendbc.car.common.conversions import Conversions as CV
from common.realtime import Ratekeeper
from selfdrive.modeld.constants import ModelConstants
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC


def nested_get(obj: Any, path: str, default: Any = None) -> Any:
  cur = obj
  for part in path.split("."):
    try:
      cur = getattr(cur, part)
    except Exception:
      return default
  return cur


def as_float(v: Any, default: float = 0.0) -> float:
  try:
    x = float(v)
    if math.isfinite(x):
      return x
  except Exception:
    pass
  return float(default)


def as_bool(v: Any, default: bool = False) -> bool:
  try:
    return bool(v)
  except Exception:
    return bool(default)


def interp_scalar(x: float, xp: list[float], fp: list[float]) -> float:
  try:
    return float(np.interp(float(x), xp, fp))
  except Exception:
    return float(fp[-1])


def compute_unity_curve_targets(model: Any, v_ego: float, factor: float = 1.0) -> tuple[float, float]:
  try:
    vel_x = list(nested_get(model, "velocity.x", []) or [])
    ori_z = list(nested_get(model, "orientationRate.z", []) or [])
    if len(vel_x) != len(ModelConstants.T_IDXS) or len(ori_z) != len(ModelConstants.T_IDXS):
      return 0.0, 0.0

    v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, vel_x)
    max_lat_accel = interp_scalar(v_ego, [5.0, 10.0, 20.0], [1.5, 2.0, 3.0])
    curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, ori_z) / np.clip(v, 0.3, 100.0)
    max_v = factor * np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
    max_v = np.clip(max_v, 0.0, 100.0)
    near = float(np.min(max_v[:6])) if len(max_v) >= 6 else float(np.min(max_v))
    tail = float(np.min(max_v))
    return near, tail
  except Exception:
    return 0.0, 0.0


def first_valid_speed_ms(car_state: Any) -> float:
  cruise = nested_get(car_state, "cruiseState", None)
  speed = as_float(nested_get(cruise, "speed", 0.0), 0.0)
  speed_cluster = as_float(nested_get(cruise, "speedCluster", 0.0), 0.0)
  return max(speed, speed_cluster)


def first_lp_speeds(lp: Any) -> tuple[float, float, float, float]:
  speeds = list(getattr(lp, "speeds", []) or [])
  if not speeds:
    return 0.0, 0.0, 0.0, 0.0

  def pick(idx: int) -> float:
    if idx < len(speeds):
      return as_float(speeds[idx], 0.0)
    return as_float(speeds[-1], 0.0)

  return pick(0), pick(4), pick(8), as_float(speeds[-1], 0.0)


def infer_blocker(
  *,
  module_active: bool,
  gas_pressed: bool,
  brake_pressed: bool,
  v_ego: float,
  stock_set_ms: float,
  controls_vcruise_ms: float,
  car_vcruise_ms: float,
  lead_status: bool,
  lead_drel: float,
  lead_vrel: float,
  lplast: float,
  lp_has_lead: bool,
  curve_near_ms: float,
  force_decel: bool,
) -> tuple[str, str]:
  if not module_active:
    return "not_active", "stock cruise / adaptive path not active"
  if gas_pressed or brake_pressed:
    return "driver_override", "gas/brake pressed"
  if force_decel:
    return "force_decel", "controlsState.forceDecel active"

  effective_ceiling_ms = max(controls_vcruise_ms, car_vcruise_ms, stock_set_ms)
  lead_constraining = bool(
    lead_status and (
      lead_drel < 45.0
      or lead_vrel < -0.25
      or lp_has_lead
    )
  )

  planner_low = bool(
    lplast > 0.1
    and effective_ceiling_ms > max(v_ego, stock_set_ms) + 0.6
    and lplast < effective_ceiling_ms - 0.6
  )

  if not planner_low:
    if lplast > stock_set_ms + 0.7 and effective_ceiling_ms > stock_set_ms + 0.7:
      return "acc_sync_block", "planner asks higher, stock set speed not following"
    return "ok_or_no_block", "no clear speed-up block"

  if lead_constraining:
    return "lead_hold", "lead/plan still constraining planner target"

  if curve_near_ms > 0.1 and curve_near_ms < effective_ceiling_ms - 0.8 and curve_near_ms < v_ego + 0.5:
    return "curve_model_hold", "Unity parse_model curve cap still below cruise ceiling"

  if controls_vcruise_ms <= stock_set_ms + 0.3 and car_vcruise_ms <= stock_set_ms + 0.3:
    return "ceiling_low", "cruise ceiling source is not above current set speed"

  return "planner_unknown_hold", "planner low on clear road without obvious lead/curve ceiling cause"


def main() -> int:
  parser = argparse.ArgumentParser(description="Second-stage cereal watcher for XNOR speed-up blocks.")
  parser.add_argument("--out", default="/data/media/0/realdata/xnor_planner_root_watch.csv")
  parser.add_argument("--rate", type=float, default=10.0)
  parser.add_argument("--curve-factor", type=float, default=1.0)
  args = parser.parse_args()

  out_path = Path(args.out)
  out_path.parent.mkdir(parents=True, exist_ok=True)

  sm = messaging.SubMaster(["carState", "controlsState", "radarState", "longitudinalPlan", "modelV2"])
  rk = Ratekeeper(int(args.rate), print_delay_threshold=None)

  fieldnames = [
    "ts_wall",
    "module_active",
    "controls_enabled",
    "stock_cruise_enabled",
    "controls_long_state",
    "stock_set_ms",
    "v_ego_ms",
    "controls_vcruise_kph",
    "car_vcruise_kph",
    "force_decel",
    "lead_status",
    "lead_drel_m",
    "lead_vrel_ms",
    "lp_has_lead",
    "lp_a_target",
    "lp_0_ms",
    "lp_4_ms",
    "lp_8_ms",
    "lp_last_ms",
    "curve_near_ms",
    "curve_tail_ms",
    "gas_pressed",
    "brake_pressed",
    "likely_blocker",
    "explanation",
  ]

  with out_path.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

    while True:
      sm.update(0)
      car_state = sm["carState"]
      controls = sm["controlsState"]
      radar = sm["radarState"]
      lp = sm["longitudinalPlan"]
      model = sm["modelV2"]

      v_ego = as_float(getattr(car_state, "vEgo", 0.0), 0.0)
      stock_set_ms = first_valid_speed_ms(car_state)
      controls_vcruise_kph = as_float(getattr(controls, "vCruise", 0.0), 0.0)
      car_vcruise_kph = as_float(getattr(car_state, "vCruise", 0.0), 0.0)

      controls_enabled = as_bool(getattr(controls, "enabled", False), False)
      controls_long_state = str(getattr(controls, "longControlState", "") or "")
      stock_cruise_enabled = as_bool(nested_get(car_state, "cruiseState.enabled", False), False)

      module_active = bool(
        stock_cruise_enabled
        or stock_set_ms > 0.1
        or car_vcruise_kph > 1.0
        or controls_vcruise_kph > 1.0
      )

      lead = nested_get(radar, "leadOne", None)
      lead_status = as_bool(nested_get(lead, "status", False), False)
      lead_drel = as_float(nested_get(lead, "dRel", 0.0), 0.0)
      lead_vrel = as_float(nested_get(lead, "vRel", 0.0), 0.0)

      lp0, lp4, lp8, lplast = first_lp_speeds(lp)
      lp_has_lead = as_bool(getattr(lp, "hasLead", False), False)
      lp_a_target = as_float(getattr(lp, "aTarget", 0.0), 0.0)

      curve_near_ms, curve_tail_ms = compute_unity_curve_targets(model, v_ego, factor=float(args.curve_factor))

      gas_pressed = as_bool(getattr(car_state, "gasPressed", False), False)
      brake_pressed = as_bool(getattr(car_state, "brakePressed", False), False)
      force_decel = as_bool(getattr(controls, "forceDecel", False), False)

      blocker, explanation = infer_blocker(
        module_active=module_active,
        gas_pressed=gas_pressed,
        brake_pressed=brake_pressed,
        v_ego=v_ego,
        stock_set_ms=stock_set_ms,
        controls_vcruise_ms=controls_vcruise_kph * CV.KPH_TO_MS,
        car_vcruise_ms=car_vcruise_kph * CV.KPH_TO_MS,
        lead_status=lead_status,
        lead_drel=lead_drel,
        lead_vrel=lead_vrel,
        lplast=lplast,
        lp_has_lead=lp_has_lead,
        curve_near_ms=curve_near_ms,
        force_decel=force_decel,
      )

      writer.writerow({
        "ts_wall": f"{time.time():.3f}",
        "module_active": int(module_active),
        "controls_enabled": int(controls_enabled),
        "stock_cruise_enabled": int(stock_cruise_enabled),
        "controls_long_state": controls_long_state,
        "stock_set_ms": f"{stock_set_ms:.3f}",
        "v_ego_ms": f"{v_ego:.3f}",
        "controls_vcruise_kph": f"{controls_vcruise_kph:.3f}",
        "car_vcruise_kph": f"{car_vcruise_kph:.3f}",
        "force_decel": int(force_decel),
        "lead_status": int(lead_status),
        "lead_drel_m": f"{lead_drel:.3f}",
        "lead_vrel_ms": f"{lead_vrel:.3f}",
        "lp_has_lead": int(lp_has_lead),
        "lp_a_target": f"{lp_a_target:.3f}",
        "lp_0_ms": f"{lp0:.3f}",
        "lp_4_ms": f"{lp4:.3f}",
        "lp_8_ms": f"{lp8:.3f}",
        "lp_last_ms": f"{lplast:.3f}",
        "curve_near_ms": f"{curve_near_ms:.3f}",
        "curve_tail_ms": f"{curve_tail_ms:.3f}",
        "gas_pressed": int(gas_pressed),
        "brake_pressed": int(brake_pressed),
        "likely_blocker": blocker,
        "explanation": explanation,
      })
      f.flush()
      rk.keep_time()


if __name__ == "__main__":
  raise SystemExit(main())
