#!/usr/bin/env python3
"""
/data/openpilot/tools/xnor_acc_debug_dump_v9.py

Columns:
ts,vEgo,carVCruiseKph,controlsVCruiseKph,followDistanceS,cruiseEnabled,leadStatus,dRel,vRel,lp_0,lp_min12,lp_last
"""

from __future__ import annotations

import argparse
import time

from cereal import messaging


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--seconds", type=int, default=120)
  ap.add_argument("--hz", type=float, default=10.0)
  args = ap.parse_args()

  sm = messaging.SubMaster(["carState", "controlsState", "radarState", "longitudinalPlan"])

  dt = 1.0 / float(args.hz)
  end_t = time.monotonic() + float(args.seconds)

  print("ts,vEgo,carVCruiseKph,controlsVCruiseKph,followDistanceS,cruiseEnabled,leadStatus,dRel,vRel,lp_0,lp_min12,lp_last")

  while time.monotonic() < end_t:
    sm.update(0)

    cs = sm["carState"]
    ctrl = sm["controlsState"]
    rs = sm["radarState"] if sm.valid.get("radarState", False) else None
    lp = sm["longitudinalPlan"] if sm.valid.get("longitudinalPlan", False) else None

    ts = time.time()

    v_ego = float(getattr(cs, "vEgo", 0.0) or 0.0)
    car_v = float(getattr(cs, "vCruise", 0.0) or 0.0)
    ctrl_v = float(getattr(ctrl, "vCruise", 0.0) or 0.0)
    fd = int(getattr(cs, "followDistanceS", 255) or 255)

    cruise = getattr(cs, "cruiseState", None)
    enabled = int(bool(getattr(cruise, "enabled", False))) if cruise is not None else 0

    lead_status = 0
    d_rel = 0.0
    v_rel = 0.0
    if rs is not None:
      lead = rs.leadOne
      lead_status = int(bool(getattr(lead, "status", False)))
      d_rel = float(getattr(lead, "dRel", 0.0) or 0.0)
      v_rel = float(getattr(lead, "vRel", 0.0) or 0.0)

    lp_0 = 0.0
    lp_min12 = 0.0
    lp_last = 0.0
    if lp is not None:
      speeds = list(getattr(lp, "speeds", []))
      if speeds:
        lp_0 = float(speeds[0])
        lp_last = float(speeds[-1])
        lp_min12 = float(min(speeds[: min(12, len(speeds))]))

    print(f"{ts:.3f},{v_ego:.3f},{car_v:.3f},{ctrl_v:.3f},{fd:d},{enabled:d},{lead_status:d},{d_rel:.3f},{v_rel:.3f},{lp_0:.3f},{lp_min12:.3f},{lp_last:.3f}")

    time.sleep(dt)

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
