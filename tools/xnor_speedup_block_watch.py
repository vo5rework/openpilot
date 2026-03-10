
#!/usr/bin/env python3
"""
Cereal/SubMaster diagnostic watcher for XNOR Tesla ACC/LONG speed-up blocking.

Goal
- Tell us whether speed-up is being blocked in:
  1) the planner/MPC path (plan never rises),
  2) the downstream stock-cruise sync path (plan rises but set speed does not),
  3) curve or lead constraints that remain active,
  4) driver override/manual input.

This script does not patch control logic.
It only subscribes to existing cereal services and writes a CSV plus a 1 Hz console summary.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from collections import deque
from pathlib import Path

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper


RUN = True


def _sig_handler(sig, frame):
  global RUN
  RUN = False


def safe_float(value, default=0.0) -> float:
  try:
    v = float(value)
    return v if math.isfinite(v) else float(default)
  except Exception:
    return float(default)


def safe_bool(value, default=False) -> bool:
  try:
    return bool(value)
  except Exception:
    return bool(default)


def get_nested(obj, path: str, default=None):
  cur = obj
  for part in path.split("."):
    try:
      cur = getattr(cur, part)
    except Exception:
      return default
  return cur


def list_or_empty(value):
  try:
    return list(value)
  except Exception:
    return []


def get_plan_speed(lp, idx: int, default=0.0) -> float:
  speeds = list_or_empty(getattr(lp, "speeds", []))
  if not speeds:
    return float(default)
  if idx < 0:
    idx = len(speeds) + idx
  if idx < 0 or idx >= len(speeds):
    return float(default)
  return safe_float(speeds[idx], default)


def compute_curve_metrics(model_v2) -> dict[str, float]:
  orientation = list_or_empty(get_nested(model_v2, "orientationRate.z", []))
  velocity_x = list_or_empty(get_nested(model_v2, "velocity.x", []))

  n = min(len(orientation), len(velocity_x))
  if n == 0:
    return {
      "model_ok": 0,
      "orientation_points": 0,
      "velocity_points": 0,
      "max_curv_ahead": 0.0,
      "max_lat_accel_ahead": 0.0,
      "curve_speed_hint": 0.0,
      "curve_detected_hint": 0,
    }

  max_curv = 0.0
  max_lat = 0.0
  for i in range(min(n, 20)):
    w = abs(safe_float(orientation[i], 0.0))
    vx = max(safe_float(velocity_x[i], 0.0), 0.1)
    curv = w / vx
    lat = w * vx
    max_curv = max(max_curv, curv)
    max_lat = max(max_lat, lat)

  # Same class of rough vision hint as the earlier watcher: not a control input, only a diagnostic hint.
  lat_accel_limit = 1.8
  curve_speed_hint = math.sqrt(lat_accel_limit / max(max_curv, 1e-6)) if max_curv > 0.0 else 0.0
  curve_detected_hint = 1 if (max_curv > 0.0022 or max_lat > 1.1) else 0

  return {
    "model_ok": 1,
    "orientation_points": len(orientation),
    "velocity_points": len(velocity_x),
    "max_curv_ahead": max_curv,
    "max_lat_accel_ahead": max_lat,
    "curve_speed_hint": curve_speed_hint,
    "curve_detected_hint": curve_detected_hint,
  }


class TrendWindow:
  def __init__(self, seconds: float = 2.0):
    self.seconds = float(seconds)
    self.samples = deque()

  def push(self, ts: float, value: float) -> None:
    self.samples.append((float(ts), float(value)))
    cutoff = float(ts) - self.seconds
    while self.samples and self.samples[0][0] < cutoff:
      self.samples.popleft()

  def delta(self) -> float:
    if len(self.samples) < 2:
      return 0.0
    return float(self.samples[-1][1]) - float(self.samples[0][1])


def infer_blocker(row: dict[str, object], set_speed_trend_ms: float, lead_drel_trend_m: float) -> tuple[str, str]:
  cruise_enabled = int(row["cruiseEnabled"]) == 1
  if not cruise_enabled:
    return "not_active", "cruise not enabled"

  if int(row["gasPressed"]) == 1 or int(row["brakePressed"]) == 1:
    return "driver_override", "gas or brake pressed"

  v_ego = safe_float(row["vEgo"])
  set_speed = safe_float(row["setSpeed"])
  controls_vc = safe_float(row["controlsVCruise"])
  car_vc = safe_float(row["carVCruise"])
  ceiling = max(controls_vc, car_vc)
  lp0 = safe_float(row["lp_0"])
  lp_last = safe_float(row["lp_last"])
  lead_status = int(row["lead1_status"]) == 1
  lead_drel = safe_float(row["lead1_dRel"])
  lead_vrel = safe_float(row["lead1_vRel"])
  lp_has_lead = int(row["lp_hasLead"]) == 1
  curve_detected = int(row["curve_detected_hint"]) == 1
  curve_speed_hint = safe_float(row["curve_speed_hint"])

  ceiling_above_set = ceiling > (set_speed + 0.35)
  planner_wants_accel = (lp_last > (set_speed + 0.25)) or (lp0 > (set_speed + 0.25))
  planner_low_vs_ceiling = ceiling_above_set and (lp_last < (ceiling - 0.35))
  set_speed_rising = set_speed_trend_ms > 0.18
  lead_opening = lead_status and ((lead_vrel > 0.20) or (lead_drel_trend_m > 1.5))
  lead_far = lead_status and lead_drel > 40.0
  curve_hold = curve_detected and (0.1 < curve_speed_hint < (set_speed - 0.30))

  if curve_hold and not lead_status:
    return "curve_hold", "curve hint still below current set speed"

  if lead_status and not lead_opening and not lead_far:
    return "lead_constraining", "lead still genuinely constraining"

  if lead_opening and lp_has_lead and planner_low_vs_ceiling:
    return "planner_low_with_opening_lead", "lead is opening but plan tail still below ceiling"

  if (not lead_status) and planner_low_vs_ceiling and not curve_hold:
    return "planner_low_clear_road", "clear road ceiling is high but plan tail is not recovering"

  if planner_wants_accel and ceiling_above_set and not set_speed_rising:
    return "downstream_not_tracking_plan", "plan wants accel but stock set speed is not rising"

  if ceiling <= (set_speed + 0.35) and lp_last > (v_ego + 0.25):
    return "ceiling_not_above_set", "planner has some headroom but cruise ceiling is not above current set speed"

  if lead_opening and not planner_wants_accel:
    return "planner_not_recovering_from_lead", "lead gap is growing but plan is not asking for accel"

  return "no_clear_block", "no obvious block on this sample"


def main() -> None:
  parser = argparse.ArgumentParser(description="XNOR cereal watcher for speed-up blocking")
  parser.add_argument("--out", default="", help="CSV output path")
  parser.add_argument("--rate", type=int, default=10, help="logging rate in Hz")
  args = parser.parse_args()

  services = ["carState", "controlsState", "radarState", "longitudinalPlan", "modelV2", "selfdriveState"]
  sm = messaging.SubMaster(services)
  rk = Ratekeeper(args.rate, None)

  out_path = Path(args.out) if args.out else Path(f"/data/media/0/realdata/xnor_speedup_watch_{time.strftime('%Y%m%d_%H%M%S')}.csv")
  out_path.parent.mkdir(parents=True, exist_ok=True)

  fields = [
    "ts",
    "frame",
    "vEgo",
    "setSpeed",
    "controlsVCruise",
    "carVCruise",
    "cruiseEnabled",
    "standstill",
    "gasPressed",
    "brakePressed",
    "followDistanceS",
    "lead1_status",
    "lead1_dRel",
    "lead1_vRel",
    "lead1_yRel",
    "lead2_status",
    "lead2_dRel",
    "lead2_vRel",
    "lead2_yRel",
    "lp_hasLead",
    "lp_aTarget",
    "lp_0",
    "lp_4",
    "lp_8",
    "lp_last",
    "model_ok",
    "orientation_points",
    "velocity_points",
    "max_curv_ahead",
    "max_lat_accel_ahead",
    "curve_speed_hint",
    "curve_detected_hint",
    "set_speed_trend_ms",
    "lead_drel_trend_m",
    "likely_blocker",
    "explanation",
  ]

  set_speed_window = TrendWindow(seconds=2.0)
  lead_drel_window = TrendWindow(seconds=2.0)

  print(f"[xnor_speedup_block_watch] writing CSV to {out_path}", flush=True)

  with out_path.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fields)
    writer.writeheader()

    last_console_print_s = 0.0
    frame = 0

    while RUN:
      sm.update(0)
      now = time.time()

      car_state = sm["carState"]
      controls_state = sm["controlsState"]
      radar_state = sm["radarState"]
      long_plan = sm["longitudinalPlan"]
      model_v2 = sm["modelV2"]

      lead1 = getattr(radar_state, "leadOne", None)
      lead2 = getattr(radar_state, "leadTwo", None)

      v_ego = safe_float(get_nested(car_state, "vEgo", 0.0))
      set_speed = safe_float(get_nested(car_state, "cruiseState.speed", 0.0))
      controls_vcruise = safe_float(get_nested(controls_state, "vCruise", 0.0))
      car_vcruise = safe_float(get_nested(car_state, "vCruise", controls_vcruise))
      cruise_enabled = 1 if safe_bool(get_nested(car_state, "cruiseState.enabled", False)) else 0
      standstill = 1 if safe_bool(get_nested(car_state, "cruiseState.standstill", False)) else 0
      gas_pressed = 1 if safe_bool(get_nested(car_state, "gasPressed", False)) else 0
      brake_pressed = 1 if safe_bool(get_nested(car_state, "brakePressed", False)) else 0
      follow_distance_s = safe_float(get_nested(car_state, "followDistanceS", 0.0))
      lp_has_lead = 1 if safe_bool(get_nested(long_plan, "hasLead", False)) else 0
      lp_a_target = safe_float(get_nested(long_plan, "aTarget", 0.0))

      curve = compute_curve_metrics(model_v2)

      set_speed_window.push(now, set_speed)
      lead_drel_window.push(now, safe_float(get_nested(lead1, "dRel", 0.0)))

      row = {
        "ts": now,
        "frame": frame,
        "vEgo": v_ego,
        "setSpeed": set_speed,
        "controlsVCruise": controls_vcruise,
        "carVCruise": car_vcruise,
        "cruiseEnabled": cruise_enabled,
        "standstill": standstill,
        "gasPressed": gas_pressed,
        "brakePressed": brake_pressed,
        "followDistanceS": follow_distance_s,
        "lead1_status": 1 if safe_bool(get_nested(lead1, "status", False)) else 0,
        "lead1_dRel": safe_float(get_nested(lead1, "dRel", 0.0)),
        "lead1_vRel": safe_float(get_nested(lead1, "vRel", 0.0)),
        "lead1_yRel": safe_float(get_nested(lead1, "yRel", 0.0)),
        "lead2_status": 1 if safe_bool(get_nested(lead2, "status", False)) else 0,
        "lead2_dRel": safe_float(get_nested(lead2, "dRel", 0.0)),
        "lead2_vRel": safe_float(get_nested(lead2, "vRel", 0.0)),
        "lead2_yRel": safe_float(get_nested(lead2, "yRel", 0.0)),
        "lp_hasLead": lp_has_lead,
        "lp_aTarget": lp_a_target,
        "lp_0": get_plan_speed(long_plan, 0, 0.0),
        "lp_4": get_plan_speed(long_plan, 4, 0.0),
        "lp_8": get_plan_speed(long_plan, 8, 0.0),
        "lp_last": get_plan_speed(long_plan, -1, 0.0),
        "model_ok": curve["model_ok"],
        "orientation_points": curve["orientation_points"],
        "velocity_points": curve["velocity_points"],
        "max_curv_ahead": curve["max_curv_ahead"],
        "max_lat_accel_ahead": curve["max_lat_accel_ahead"],
        "curve_speed_hint": curve["curve_speed_hint"],
        "curve_detected_hint": curve["curve_detected_hint"],
        "set_speed_trend_ms": set_speed_window.delta(),
        "lead_drel_trend_m": lead_drel_window.delta(),
      }

      blocker, explanation = infer_blocker(row, row["set_speed_trend_ms"], row["lead_drel_trend_m"])
      row["likely_blocker"] = blocker
      row["explanation"] = explanation

      writer.writerow(row)
      f.flush()

      if now - last_console_print_s >= 1.0:
        print(
          "[xnor_speedup_block_watch] "
          f"blk={blocker} vEgo={row['vEgo']:.2f} set={row['setSpeed']:.2f} "
          f"ctrlVc={row['controlsVCruise']:.2f} carVc={row['carVCruise']:.2f} "
          f"lp0={row['lp_0']:.2f} lplast={row['lp_last']:.2f} "
          f"lead=({row['lead1_status']},{row['lead1_dRel']:.1f},{row['lead1_vRel']:.2f}) "
          f"curveV={row['curve_speed_hint']:.2f} note={explanation}",
          flush=True,
        )
        last_console_print_s = now

      frame += 1
      rk.keep_time()

  print(f"[xnor_speedup_block_watch] done: {out_path}", flush=True)


if __name__ == "__main__":
  signal.signal(signal.SIGINT, _sig_handler)
  signal.signal(signal.SIGTERM, _sig_handler)
  main()
