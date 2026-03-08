#!/usr/bin/env python3
"""
tools/tesla_curve_watch.py

Live curve watcher for Tesla/XNOR/OpenPilot longitudinal turn tracing.

Logs the same family of metrics used by the Unity/XNOR planner curve path:
- curve_area
- max_curv_ahead
- far_curv_peak
- anticipatory_slowdown
- curve_detected
- max_v_curve
- filtered curve speed
- current lead / plan / cruise state

This does not change behavior. It only watches and exports CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List

import numpy as np

from cereal import messaging
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.common.conversions import Conversions as CV


AREA_THRESHOLD = 0.012
CURVE_PEAK_THRESHOLD = 0.0020
VISION_CURVE_TARGET_LAT_A = 2.1

T_IDXS_MPC = np.array([0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8,
                       2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8])


@dataclass
class FirstOrderFilterLite:
    x: float
    rc: float
    dt: float

    def update(self, new_x: float) -> float:
        alpha = self.dt / (self.rc + self.dt) if self.rc > 0.0 else 1.0
        self.x = self.x + alpha * (new_x - self.x)
        return self.x


def _safe_len(seq: object) -> int:
    try:
        return len(seq)  # type: ignore[arg-type]
    except Exception:
        return 0


def _safe_list(seq: object) -> list:
    try:
        return list(seq)  # type: ignore[arg-type]
    except Exception:
        return []


def _rate_limited_update(filt: FirstOrderFilterLite, target: float, rc_up: float, rc_down: float) -> float:
    filt.rc = rc_down if target < filt.x else rc_up
    return filt.update(target)


def _get_speed_error(model_msg, v_ego: float) -> float:
    try:
        temporal_pose = getattr(model_msg, "temporalPose", None)
        if temporal_pose is not None:
            trans = getattr(temporal_pose, "trans", [])
            if _safe_len(trans):
                return float(np.clip(float(trans[0]) - float(v_ego), -5.0, 5.0))
    except Exception:
        pass

    try:
        vel_x = getattr(model_msg.velocity, "x", [])
        if _safe_len(vel_x):
            return float(np.clip(float(vel_x[0]) - float(v_ego), -5.0, 5.0))
    except Exception:
        pass

    return 0.0


def compute_curve_metrics(model_msg, v_ego: float, model_error: float, v_turn_filter: FirstOrderFilterLite) -> dict:
    metrics = {
        "model_points_ok": 0,
        "orientation_points_ok": 0,
        "curve_area": 0.0,
        "max_curv_ahead": 0.0,
        "far_curv_peak": 0.0,
        "anticipatory_slowdown": 1.0,
        "curve_detected": 0,
        "max_v_curve": 0.0,
        "v_curve_filtered": float(v_turn_filter.x),
        "v_corrected_0": 0.0,
    }

    pos_x = _safe_list(getattr(getattr(model_msg, "position", None), "x", []))
    vel_x = _safe_list(getattr(getattr(model_msg, "velocity", None), "x", []))
    acc_x = _safe_list(getattr(getattr(model_msg, "acceleration", None), "x", []))
    or_z = _safe_list(getattr(getattr(model_msg, "orientationRate", None), "z", []))

    if not (len(pos_x) == ModelConstants.IDX_N and len(vel_x) == ModelConstants.IDX_N and len(acc_x) == ModelConstants.IDX_N):
        return metrics

    metrics["model_points_ok"] = 1
    v_raw = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, vel_x)
    v_corrected = np.maximum(v_raw, max(0.0, float(v_ego) - 1.5)) - float(model_error)
    metrics["v_corrected_0"] = float(v_corrected[0])

    if len(or_z) != ModelConstants.IDX_N:
        return metrics

    metrics["orientation_points_ok"] = 1
    raw_curv = np.abs(np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, or_z)) / np.clip(v_corrected, 0.3, 100.0)
    num_idx = len(raw_curv)

    far_start = min(10, max(0, num_idx - 1))
    far_end = min(32, num_idx)
    far_curv_peak = float(np.max(raw_curv[far_start:far_end])) if far_end > far_start else 0.0
    anticipatory_slowdown = float(np.interp(far_curv_peak, [0.0008, 0.003], [1.0, 0.78]))

    curve_area = float(np.sum(raw_curv[:25]) * 0.2)
    max_curv_ahead = float(np.max(raw_curv[2:20])) if num_idx > 2 else 0.0
    curve_detected = int((curve_area > AREA_THRESHOLD) or (max_curv_ahead > CURVE_PEAK_THRESHOLD))

    max_v_curve = 0.0
    v_curve_filtered = float(v_turn_filter.x)
    if curve_detected:
        lat_stress_factor = float((float(v_ego) ** 2) * max_curv_ahead)
        torque_multiplier = float(np.interp(lat_stress_factor, [0.015, 0.05], [1.0, 0.65]))
        dynamic_multiplier = float(np.interp(max_curv_ahead, [0.0015, 0.008], [1.0, 0.70]))
        max_v_curve = dynamic_multiplier * torque_multiplier * anticipatory_slowdown * math.sqrt(max(VISION_CURVE_TARGET_LAT_A / (max_curv_ahead + 1e-4), 0.0))
        v_curve_filtered = float(_rate_limited_update(v_turn_filter, max_v_curve, rc_up=0.3, rc_down=0.05))
    else:
        if anticipatory_slowdown < 1.0:
            v_curve_filtered = float(_rate_limited_update(v_turn_filter, float(v_corrected[0]) * anticipatory_slowdown, rc_up=0.3, rc_down=0.05))
        else:
            v_curve_filtered = float(_rate_limited_update(v_turn_filter, float(v_corrected[0]), rc_up=0.3, rc_down=0.05))

    metrics.update({
        "curve_area": curve_area,
        "max_curv_ahead": max_curv_ahead,
        "far_curv_peak": far_curv_peak,
        "anticipatory_slowdown": anticipatory_slowdown,
        "curve_detected": curve_detected,
        "max_v_curve": max_v_curve,
        "v_curve_filtered": v_curve_filtered,
    })
    return metrics


def build_row(sm, curve_metrics: dict) -> dict:
    cs = sm["carState"]
    controls = sm["controlsState"]
    selfdrive = sm["selfdriveState"]
    model = sm["modelV2"]
    radar = sm["radarState"]
    plan = sm["longitudinalPlan"]

    lead = getattr(radar, "leadOne", None)
    lead_status = int(getattr(lead, "status", False)) if lead is not None else 0
    lead_d_rel = float(getattr(lead, "dRel", 0.0) or 0.0) if lead is not None else 0.0
    lead_v_rel = float(getattr(lead, "vRel", 0.0) or 0.0) if lead is not None else 0.0
    lead_y_rel = float(getattr(lead, "yRel", 0.0) or 0.0) if lead is not None else 0.0

    try:
        plan_speeds = list(plan.speeds)
    except Exception:
        plan_speeds = []

    try:
        plan_accels = list(plan.accels)
    except Exception:
        plan_accels = []

    try:
        plan_jerks = list(plan.jerks)
    except Exception:
        plan_jerks = []

    try:
        model_or_z = list(model.orientationRate.z)
    except Exception:
        model_or_z = []

    row = {
        "ts_wall": time.time(),
        "v_ego_mps": float(getattr(cs, "vEgo", 0.0) or 0.0),
        "a_ego_mps2": float(getattr(cs, "aEgo", 0.0) or 0.0),
        "v_cruise_kph": float(getattr(cs, "vCruise", 0.0) or 0.0),
        "controls_enabled": int(bool(getattr(controls, "enabled", getattr(selfdrive, "enabled", False)))),
        "experimental_mode": int(bool(getattr(controls, "experimentalMode", getattr(selfdrive, "experimentalMode", False)))),
        "long_control_state": int(getattr(controls, "longControlState", LongCtrlState.off)),
        "follow_distance_s": int(getattr(cs, "followDistanceS", -1)),
        "lead_status": lead_status,
        "lead_d_rel_m": lead_d_rel,
        "lead_v_rel_mps": lead_v_rel,
        "lead_y_rel_m": lead_y_rel,
        "model_or_z_len": len(model_or_z),
        "model_or_z_0": float(model_or_z[0]) if model_or_z else 0.0,
        "plan_speed_0_mps": float(plan_speeds[0]) if plan_speeds else 0.0,
        "plan_speed_last_mps": float(plan_speeds[-1]) if plan_speeds else 0.0,
        "plan_accel_0_mps2": float(plan_accels[0]) if plan_accels else 0.0,
        "plan_jerk_0_mps3": float(plan_jerks[0]) if plan_jerks else 0.0,
    }
    row.update(curve_metrics)
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description="Watch live curve metrics and export CSV.")
    parser.add_argument("--seconds", type=float, default=120.0, help="Capture duration in seconds.")
    parser.add_argument("--output", type=str, default="", help="CSV output path. Default: /data/media/0/realdata/curve_watch_<ts>.csv")
    parser.add_argument("--print-events", action="store_true", help="Print when curve_detected changes or curve speed drops materially.")
    args = parser.parse_args()

    out_path = Path(args.output) if args.output else Path(f"/data/media/0/realdata/curve_watch_{int(time.time())}.csv")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    services = ["carState", "controlsState", "selfdriveState", "modelV2", "radarState", "longitudinalPlan"]
    sm = messaging.SubMaster(services)
    stop = {"flag": False}

    def _handle_stop(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    v_turn_filter = FirstOrderFilterLite(0.0, rc=0.2, dt=0.05)

    fieldnames = [
        "ts_wall",
        "v_ego_mps", "a_ego_mps2", "v_cruise_kph",
        "controls_enabled", "experimental_mode", "long_control_state", "follow_distance_s",
        "lead_status", "lead_d_rel_m", "lead_v_rel_mps", "lead_y_rel_m",
        "model_or_z_len", "model_or_z_0",
        "curve_area", "max_curv_ahead", "far_curv_peak", "anticipatory_slowdown",
        "curve_detected", "max_v_curve", "v_curve_filtered", "v_corrected_0",
        "model_points_ok", "orientation_points_ok",
        "plan_speed_0_mps", "plan_speed_last_mps", "plan_accel_0_mps2", "plan_jerk_0_mps3",
    ]

    last_curve_detected = None
    last_print_ts = 0.0
    start = time.monotonic()

    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        while not stop["flag"] and (time.monotonic() - start) < float(args.seconds):
            sm.update(0)

            if not sm.updated["modelV2"]:
                time.sleep(0.01)
                continue

            v_ego = float(getattr(sm["carState"], "vEgo", 0.0) or 0.0)
            model_error = _get_speed_error(sm["modelV2"], v_ego)
            curve_metrics = compute_curve_metrics(sm["modelV2"], v_ego, model_error, v_turn_filter)
            row = build_row(sm, curve_metrics)
            writer.writerow(row)

            if args.print_events:
                now = time.monotonic()
                curve_detected = row["curve_detected"]
                trigger_print = False
                if last_curve_detected is None or curve_detected != last_curve_detected:
                    trigger_print = True
                if (row["v_curve_filtered"] + 0.5) < row["plan_speed_last_mps"]:
                    trigger_print = True
                if trigger_print and (now - last_print_ts) > 0.2:
                    print(
                        f"curve={curve_detected} "
                        f"vEgo={row['v_ego_mps']*CV.MS_TO_MPH:.1f}mph "
                        f"plan={row['plan_speed_last_mps']*CV.MS_TO_MPH:.1f}mph "
                        f"curve_v={row['v_curve_filtered']*CV.MS_TO_MPH:.1f}mph "
                        f"area={row['curve_area']:.4f} peak={row['max_curv_ahead']:.5f} "
                        f"lead={row['lead_status']} dRel={row['lead_d_rel_m']:.1f} vRel={row['lead_v_rel_mps']:.2f}"
                    )
                    last_print_ts = now
                last_curve_detected = curve_detected

    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
