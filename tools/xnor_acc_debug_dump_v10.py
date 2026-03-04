from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass

from cereal import messaging

try:
  from openpilot.common.params import Params  # preferred in newer trees
except Exception:
  from common.params import Params  # fallback


@dataclass(frozen=True)
class TeslaFlags:
  radar_filtering: int
  lead_selection: int
  set_speed_ownership: int
  acc_blend: int


def _truthy_param_value(v: bytes | None) -> int:
  if v is None:
    return 0
  s = v.strip()
  if not s:
    return 0
  return 1 if s in (b"1", b"true", b"True", b"TRUE", b"y", b"Y", b"yes", b"YES") else 0


def read_tesla_flags(p: Params) -> TeslaFlags:
  def safe_get(key: str) -> int:
    try:
      return _truthy_param_value(p.get(key))
    except Exception:
      return 0

  return TeslaFlags(
    radar_filtering=safe_get("TeslaRadarFilteringEnabled"),
    lead_selection=safe_get("TeslaLeadSelectionEnabled"),
    set_speed_ownership=safe_get("TeslaSetSpeedOwnershipEnabled"),
    acc_blend=safe_get("TeslaAccBlendEnabled"),
  )


def _float(x, default: float = 0.0) -> float:
  try:
    if x is None:
      return default
    return float(x)
  except Exception:
    return default


def _int(x, default: int = 0) -> int:
  try:
    if x is None:
      return default
    return int(x)
  except Exception:
    return default


def _lead_fields(lead) -> tuple[int, float, float, float]:
  if lead is None:
    return 0, 0.0, 0.0, 0.0
  status = int(bool(getattr(lead, "status", False)))
  d_rel = _float(getattr(lead, "dRel", 0.0))
  v_rel = _float(getattr(lead, "vRel", 0.0))
  y_rel = _float(getattr(lead, "yRel", 0.0))
  return status, d_rel, v_rel, y_rel


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--seconds", type=int, default=180, help="How long to dump")
  ap.add_argument("--hz", type=float, default=10.0, help="Dump rate (Hz)")
  ap.add_argument("--out", type=str, default="", help="Optional output file path (append). If empty, prints to stdout.")
  args = ap.parse_args()

  dt = 1.0 / float(args.hz)
  end_t = time.monotonic() + float(args.seconds)

  sm = messaging.SubMaster(["carState", "controlsState", "radarState", "longitudinalPlan"])
  p = Params()

  header = (
    "ts,"
    "vEgo,"
    "carVCruiseKph,controlsVCruiseKph,"
    "followDistanceS,cruiseEnabled,"
    "lead1_status,lead1_dRel,lead1_vRel,lead1_yRel,"
    "lead2_status,lead2_dRel,lead2_vRel,lead2_yRel,"
    "lp_0,lp_min12,lp_last,"
    "ff_radar_filtering,ff_lead_selection,ff_set_speed_ownership,ff_acc_blend,"
    "lead_ignored_hint"
  )

  out_fh = None
  if args.out:
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    out_fh = open(args.out, "a", buffering=1)
    if out_fh.tell() == 0:
      out_fh.write(header + "\n")
  else:
    print(header)

  while time.monotonic() < end_t:
    sm.update(0)

    cs = sm["carState"]
    ctrl = sm["controlsState"]
    rs = sm["radarState"] if sm.valid.get("radarState", False) else None
    lp = sm["longitudinalPlan"] if sm.valid.get("longitudinalPlan", False) else None

    flags = read_tesla_flags(p)

    ts = time.time()
    v_ego = _float(getattr(cs, "vEgo", 0.0))
    car_v = _float(getattr(cs, "vCruise", 0.0))
    ctrl_v = _float(getattr(ctrl, "vCruise", 0.0))
    fd = _int(getattr(cs, "followDistanceS", 255), 255)

    cruise = getattr(cs, "cruiseState", None)
    cruise_enabled = int(bool(getattr(cruise, "enabled", False))) if cruise is not None else 0

    lead1_status = lead1_d = lead1_v = lead1_y = 0.0
    lead2_status = lead2_d = lead2_v = lead2_y = 0.0
    lead1_status_i = 0
    lead2_status_i = 0

    if rs is not None:
      lead1_status_i, lead1_d, lead1_v, lead1_y = _lead_fields(getattr(rs, "leadOne", None))
      lead2_status_i, lead2_d, lead2_v, lead2_y = _lead_fields(getattr(rs, "leadTwo", None))

    lp_0 = lp_min12 = lp_last = 0.0
    if lp is not None:
      speeds = list(getattr(lp, "speeds", []))
      if speeds:
        lp_0 = _float(speeds[0])
        lp_last = _float(speeds[-1])
        lp_min12 = float(min(map(float, speeds[: min(12, len(speeds))])))

    # Heuristic: lead exists + ACC enabled, but planner isn't trying to go slower OR faster
    # This is only a hint; it helps spot "lead ignored" in logs quickly.
    lead_present = (lead1_status_i == 1) or (lead2_status_i == 1)
    # If lead present and vRel negative but lp_min12 not below current speed by meaningful margin, maybe ignored
    lead_vrel = lead1_v if lead1_status_i else (lead2_v if lead2_status_i else 0.0)
    hint = 0
    if cruise_enabled and lead_present:
      if lead_vrel < -0.3 and (lp_min12 >= (v_ego - 0.2)):
        hint = 1  # lead slowing but plan not reducing
      if lead_vrel > 0.3 and (lp_0 <= (v_ego + 0.2)) and (car_v > 0.0) and (ctrl_v > 0.0):
        hint = 2  # lead pulling away but plan not increasing (common "locked" symptom)

    line = (
      f"{ts:.3f},"
      f"{v_ego:.3f},"
      f"{car_v:.3f},{ctrl_v:.3f},"
      f"{fd:d},{cruise_enabled:d},"
      f"{lead1_status_i:d},{lead1_d:.3f},{lead1_v:.3f},{lead1_y:.3f},"
      f"{lead2_status_i:d},{lead2_d:.3f},{lead2_v:.3f},{lead2_y:.3f},"
      f"{lp_0:.3f},{lp_min12:.3f},{lp_last:.3f},"
      f"{flags.radar_filtering:d},{flags.lead_selection:d},{flags.set_speed_ownership:d},{flags.acc_blend:d},"
      f"{hint:d}"
    )

    if out_fh is not None:
      out_fh.write(line + "\n")
    else:
      print(line)

    time.sleep(dt)

  if out_fh is not None:
    out_fh.close()

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
