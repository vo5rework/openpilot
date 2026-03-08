from __future__ import annotations

import argparse
import math
import os
import time

from cereal import messaging


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


def _bool(x) -> int:
  try:
    return int(bool(x))
  except Exception:
    return 0


def _list_floats(seq) -> list[float]:
  try:
    return [float(x) for x in seq]
  except Exception:
    return []


def _lead_fields(lead) -> tuple[int, float, float, float]:
  if lead is None:
    return 0, 0.0, 0.0, 0.0
  status = int(bool(getattr(lead, "status", False)))
  d_rel = _float(getattr(lead, "dRel", 0.0))
  v_rel = _float(getattr(lead, "vRel", 0.0))
  y_rel = _float(getattr(lead, "yRel", 0.0))
  return status, d_rel, v_rel, y_rel


def _exp_mode(ctrl, selfdrive_state) -> int:
  v = None
  try:
    v = getattr(ctrl, "experimentalMode", None)
  except Exception:
    v = None
  if v is None:
    try:
      v = getattr(selfdrive_state, "experimentalMode", None)
    except Exception:
      v = None
  return _bool(v)


def _v_cruise(ctrl, car_state) -> float:
  v = _float(getattr(ctrl, "vCruise", 0.0), 0.0)
  if v > 0.0:
    return v
  return _float(getattr(car_state, "vCruise", 0.0), 0.0)


class CurveWatch:
  def __init__(self, target_lat_accel: float, area_threshold: float, peak_threshold: float,
               far_peak_threshold: float, fall_gain: float, rise_gain: float) -> None:
    self.target_lat_accel = float(target_lat_accel)
    self.area_threshold = float(area_threshold)
    self.peak_threshold = float(peak_threshold)
    self.far_peak_threshold = float(far_peak_threshold)
    self.fall_gain = float(fall_gain)
    self.rise_gain = float(rise_gain)
    self.v_turn_filter = 0.0
    self.last_curve_detected = 0

  def compute(self, model, v_ego: float) -> dict[str, float | int]:
    orient = getattr(model, "orientationRate", None)
    vel = getattr(model, "velocity", None)

    z = []
    vx = []
    if orient is not None:
      try:
        z = _list_floats(getattr(orient, "z", []))
      except Exception:
        z = []
    if vel is not None:
      try:
        vx = _list_floats(getattr(vel, "x", []))
      except Exception:
        vx = []

    n = min(len(z), len(vx))
    if n <= 0:
      return {
        "model_ok": 0,
        "orientation_points": len(z),
        "velocity_points": len(vx),
        "curve_area": 0.0,
        "max_curv_ahead": 0.0,
        "far_curv_peak": 0.0,
        "anticipatory_slowdown": 1.0,
        "curve_detected": 0,
        "max_v_curve": 0.0,
        "v_curve_filtered": self.v_turn_filter,
      }

    curvatures = []
    for i in range(n):
      v = max(abs(vx[i]), 0.1)
      curvatures.append(abs(z[i]) / v)

    near = curvatures[: min(n, 16)]
    far = curvatures[min(16, n): min(n, 32)]
    curve_area = sum(near)
    max_curv_ahead = max(near) if near else 0.0
    far_curv_peak = max(far) if far else 0.0

    anticipatory_slowdown = 1.0
    if far_curv_peak > self.far_peak_threshold:
      anticipatory_slowdown = max(0.55, 1.0 - min(0.35, far_curv_peak * 60.0))

    curve_detected = int(
      (curve_area > self.area_threshold) or
      (max_curv_ahead > self.peak_threshold) or
      (far_curv_peak > self.far_peak_threshold)
    )

    peak = max(max_curv_ahead, far_curv_peak * 0.9, 1e-6)
    max_v_curve = math.sqrt(max(self.target_lat_accel / peak, 0.0)) * anticipatory_slowdown

    if self.v_turn_filter <= 0.0:
      self.v_turn_filter = max_v_curve
    else:
      if max_v_curve < self.v_turn_filter:
        self.v_turn_filter += self.fall_gain * (max_v_curve - self.v_turn_filter)
      else:
        self.v_turn_filter += self.rise_gain * (max_v_curve - self.v_turn_filter)

    self.v_turn_filter = max(self.v_turn_filter, 0.0)

    return {
      "model_ok": 1,
      "orientation_points": len(z),
      "velocity_points": len(vx),
      "curve_area": curve_area,
      "max_curv_ahead": max_curv_ahead,
      "far_curv_peak": far_curv_peak,
      "anticipatory_slowdown": anticipatory_slowdown,
      "curve_detected": curve_detected,
      "max_v_curve": max_v_curve,
      "v_curve_filtered": self.v_turn_filter,
    }


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--seconds", type=int, default=180, help="How long to dump")
  ap.add_argument("--hz", type=float, default=10.0, help="Dump rate (Hz)")
  ap.add_argument("--out", type=str, default="", help="Optional output file path (append). If empty, writes to /data/media/0/realdata/")
  ap.add_argument("--print-events", action="store_true", help="Print curve enter/exit events")
  ap.add_argument("--target-lat-accel", type=float, default=1.9)
  ap.add_argument("--area-threshold", type=float, default=0.10)
  ap.add_argument("--peak-threshold", type=float, default=0.0015)
  ap.add_argument("--far-peak-threshold", type=float, default=0.0010)
  ap.add_argument("--fall-gain", type=float, default=0.35, help="How quickly filtered curve speed falls")
  ap.add_argument("--rise-gain", type=float, default=0.08, help="How quickly filtered curve speed rises")
  args = ap.parse_args()

  out = args.out.strip()
  if not out:
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = f"/data/media/0/realdata/curve_watch_{ts}.csv"

  os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

  sm = messaging.SubMaster([
    "carState",
    "controlsState",
    "selfdriveState",
    "radarState",
    "longitudinalPlan",
    "modelV2",
  ])

  cw = CurveWatch(
    target_lat_accel=args.target_lat_accel,
    area_threshold=args.area_threshold,
    peak_threshold=args.peak_threshold,
    far_peak_threshold=args.far_peak_threshold,
    fall_gain=args.fall_gain,
    rise_gain=args.rise_gain,
  )

  header = (
    "ts,"
    "vEgo,vCruise,carVCruise,followDistanceS,cruiseEnabled,experimentalMode,"
    "lead1_status,lead1_dRel,lead1_vRel,lead1_yRel,"
    "lead2_status,lead2_dRel,lead2_vRel,lead2_yRel,"
    "lp_0,lp_min12,lp_last,"
    "model_ok,orientation_points,velocity_points,"
    "curve_area,max_curv_ahead,far_curv_peak,anticipatory_slowdown,curve_detected,max_v_curve,v_curve_filtered,"
    "curve_should_slow_hint,plan_not_dropping_hint"
  )

  with open(out, "a", buffering=1) as fh:
    if fh.tell() == 0:
      fh.write(header + "\n")

    dt = 1.0 / max(float(args.hz), 0.1)
    end_t = time.monotonic() + float(args.seconds)

    while time.monotonic() < end_t:
      sm.update(0)

      cs = sm["carState"]
      ctrl = sm["controlsState"]
      sds = sm["selfdriveState"] if sm.valid.get("selfdriveState", False) else None
      rs = sm["radarState"] if sm.valid.get("radarState", False) else None
      lp = sm["longitudinalPlan"] if sm.valid.get("longitudinalPlan", False) else None
      model = sm["modelV2"] if sm.valid.get("modelV2", False) else None

      ts = time.time()
      v_ego = _float(getattr(cs, "vEgo", 0.0))
      v_cruise = _v_cruise(ctrl, cs)
      car_v = _float(getattr(cs, "vCruise", 0.0))
      fd = _int(getattr(cs, "followDistanceS", 255), 255)

      cruise = getattr(cs, "cruiseState", None)
      cruise_enabled = int(bool(getattr(cruise, "enabled", False))) if cruise is not None else 0
      exp_mode = _exp_mode(ctrl, sds)

      lead1_status_i = lead2_status_i = 0
      lead1_d = lead1_v = lead1_y = 0.0
      lead2_d = lead2_v = lead2_y = 0.0
      if rs is not None:
        lead1_status_i, lead1_d, lead1_v, lead1_y = _lead_fields(getattr(rs, "leadOne", None))
        lead2_status_i, lead2_d, lead2_v, lead2_y = _lead_fields(getattr(rs, "leadTwo", None))

      lp_0 = lp_min12 = lp_last = 0.0
      if lp is not None:
        speeds = list(getattr(lp, "speeds", []))
        if speeds:
          vals = [float(x) for x in speeds]
          lp_0 = vals[0]
          lp_last = vals[-1]
          lp_min12 = float(min(vals[: min(12, len(vals))]))

      curve = cw.compute(model, v_ego) if model is not None else {
        "model_ok": 0,
        "orientation_points": 0,
        "velocity_points": 0,
        "curve_area": 0.0,
        "max_curv_ahead": 0.0,
        "far_curv_peak": 0.0,
        "anticipatory_slowdown": 1.0,
        "curve_detected": 0,
        "max_v_curve": 0.0,
        "v_curve_filtered": cw.v_turn_filter,
      }

      lead_present = (lead1_status_i == 1) or (lead2_status_i == 1)
      curve_should_slow_hint = 0
      if int(curve["curve_detected"]) == 1 and v_ego > 8.0:
        if float(curve["v_curve_filtered"]) < (v_ego - 0.5):
          curve_should_slow_hint = 1

      plan_not_dropping_hint = 0
      if curve_should_slow_hint and not lead_present:
        if lp_min12 >= (v_ego - 0.2):
          plan_not_dropping_hint = 1

      if args.print_events and int(curve["curve_detected"]) != cw.last_curve_detected:
        state = "ENTER" if int(curve["curve_detected"]) else "EXIT"
        print(
          f"[{time.strftime('%H:%M:%S')}] CURVE_{state} "
          f"vEgo={v_ego:.2f} vCruise={v_cruise:.2f} "
          f"curve_area={float(curve['curve_area']):.4f} "
          f"max_curv={float(curve['max_curv_ahead']):.5f} "
          f"far_peak={float(curve['far_curv_peak']):.5f} "
          f"v_curve={float(curve['v_curve_filtered']):.2f} "
          f"lp_min12={lp_min12:.2f} lead={int(lead_present)}"
        )
      cw.last_curve_detected = int(curve["curve_detected"])

      line = (
        f"{ts:.3f},"
        f"{v_ego:.3f},{v_cruise:.3f},{car_v:.3f},{fd:d},{cruise_enabled:d},{exp_mode:d},"
        f"{lead1_status_i:d},{lead1_d:.3f},{lead1_v:.3f},{lead1_y:.3f},"
        f"{lead2_status_i:d},{lead2_d:.3f},{lead2_v:.3f},{lead2_y:.3f},"
        f"{lp_0:.3f},{lp_min12:.3f},{lp_last:.3f},"
        f"{int(curve['model_ok']):d},{int(curve['orientation_points']):d},{int(curve['velocity_points']):d},"
        f"{float(curve['curve_area']):.6f},{float(curve['max_curv_ahead']):.6f},{float(curve['far_curv_peak']):.6f},"
        f"{float(curve['anticipatory_slowdown']):.3f},{int(curve['curve_detected']):d},{float(curve['max_v_curve']):.3f},{float(curve['v_curve_filtered']):.3f},"
        f"{curve_should_slow_hint:d},{plan_not_dropping_hint:d}"
      )
      fh.write(line + "\n")
      time.sleep(dt)

  print(f"Wrote CSV to: {out}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
