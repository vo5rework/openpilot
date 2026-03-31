#!/usr/bin/env python3
# /data/openpilot/tools/planner_origin_watch.py
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from cereal import messaging


RUNNING = True


def _sig_handler(signum, frame) -> None:
  global RUNNING
  RUNNING = False


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _wall_iso() -> str:
  return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def _safe_float(value: Any, default: float = 0.0) -> float:
  try:
    v = float(value)
  except Exception:
    return float(default)
  return v if math.isfinite(v) else float(default)


def _safe_bool(value: Any) -> bool:
  try:
    return bool(value)
  except Exception:
    return False


def _take_numeric(seq: Any, limit: int) -> list[float]:
  out: list[float] = []
  if not seq:
    return out
  try:
    for item in list(seq)[:limit]:
      v = _safe_float(item, float("nan"))
      if math.isfinite(v):
        out.append(float(v))
  except Exception:
    return out
  return out


def _take_nested_numeric(obj: Any, attr: str, limit: int) -> list[float]:
  return _take_numeric(getattr(obj, attr, None), limit)


def _plan_summary(lp: Any) -> dict[str, Any]:
  speeds = _take_nested_numeric(lp, "speeds", 33)
  accels = _take_nested_numeric(lp, "accels", 20)
  jerks = _take_nested_numeric(lp, "jerks", 20)

  p_last = float(speeds[-1]) if speeds else None

  near_window = max(3, int(math.ceil(len(speeds) * 0.35))) if speeds else 0
  preview_window = max(5, int(math.ceil(len(speeds) * 0.70))) if speeds else 0

  p_near = min(speeds[:near_window]) if near_window > 0 else None
  p_preview = min(speeds[:preview_window]) if preview_window > 0 else None

  out = {
    "hasLead": _safe_bool(getattr(lp, "hasLead", False)),
    "aTarget": _safe_float(getattr(lp, "aTarget", 0.0)),
    "vCruise": _safe_float(getattr(lp, "vCruise", 0.0)),
    "vCruiseCluster": _safe_float(getattr(lp, "vCruiseCluster", 0.0)),
    "desiredTF": _safe_float(getattr(lp, "desiredFollowDistance", 0.0)),
    "speeds": speeds,
    "accels": accels,
    "jerks": jerks,
    "p_last": p_last,
    "p_near": p_near,
    "p_preview": p_preview,
  }

  for name in (
    "longitudinalPlanSource",
    "xState",
    "trafficState",
    "personality",
    "fcw",
  ):
    try:
      out[name] = getattr(lp, name)
    except Exception:
      pass

  return out


def _lead_summary(lead: Any) -> dict[str, Any]:
  if lead is None:
    return {"status": False}
  out = {
    "status": _safe_bool(getattr(lead, "status", False)),
    "dRel": _safe_float(getattr(lead, "dRel", 0.0)),
    "yRel": _safe_float(getattr(lead, "yRel", 0.0)),
    "vRel": _safe_float(getattr(lead, "vRel", 0.0)),
    "aRel": _safe_float(getattr(lead, "aRel", 0.0)),
    "vLead": _safe_float(getattr(lead, "vLead", 0.0)),
    "vLeadK": _safe_float(getattr(lead, "vLeadK", 0.0)),
    "aLeadK": _safe_float(getattr(lead, "aLeadK", 0.0)),
    "fcw": _safe_bool(getattr(lead, "fcw", False)),
    "modelProb": _safe_float(getattr(lead, "modelProb", 0.0)),
    "radar": _safe_bool(getattr(lead, "radar", False)),
  }
  for name in ("radarTrackId", "source"):
    try:
      out[name] = getattr(lead, name)
    except Exception:
      pass
  return out


def _car_state_summary(cs: Any) -> dict[str, Any]:
  cruise = getattr(cs, "cruiseState", None)
  out = {
    "vEgo": _safe_float(getattr(cs, "vEgo", 0.0)),
    "aEgo": _safe_float(getattr(cs, "aEgo", 0.0)),
    "standstill": _safe_bool(getattr(cs, "standstill", False)),
    "gasPressed": _safe_bool(getattr(cs, "gasPressed", False)),
    "brakePressed": _safe_bool(getattr(cs, "brakePressed", False)),
    "steeringAngleDeg": _safe_float(getattr(cs, "steeringAngleDeg", 0.0)),
    "steeringPressed": _safe_bool(getattr(cs, "steeringPressed", False)),
    "leftBlinker": _safe_bool(getattr(cs, "leftBlinker", False)),
    "rightBlinker": _safe_bool(getattr(cs, "rightBlinker", False)),
  }
  if cruise is not None:
    out["cruiseState"] = {
      "enabled": _safe_bool(getattr(cruise, "enabled", False)),
      "available": _safe_bool(getattr(cruise, "available", False)),
      "standstill": _safe_bool(getattr(cruise, "standstill", False)),
      "speed": _safe_float(getattr(cruise, "speed", 0.0)),
      "speedCluster": _safe_float(getattr(cruise, "speedCluster", 0.0)),
    }
  return out


def _controls_state_summary(cs: Any) -> dict[str, Any]:
  out = {}
  for name in (
    "enabled",
    "active",
    "vCruise",
    "vCruiseCluster",
    "longControlState",
    "forceDecel",
    "alertSize",
    "alertStatus",
  ):
    try:
      value = getattr(cs, name)
    except Exception:
      continue
    if isinstance(value, (bool, int, str)):
      out[name] = value
    else:
      out[name] = _safe_float(value, 0.0)
  return out


def _mapd_summary(mo: Any) -> dict[str, Any]:
  out = {}
  for name in (
    "suggestedSpeed",
    "mapCurveSpeed",
    "visionCurveSpeed",
    "speedLimit",
    "speedLimitAhead",
    "speedLimitAheadDistance",
    "turnSpeedLimit",
    "turnSpeedLimitEndDistance",
    "currentRoadType",
    "currentRoadName",
  ):
    try:
      value = getattr(mo, name)
    except Exception:
      continue
    if isinstance(value, str):
      out[name] = value
    else:
      out[name] = _safe_float(value, 0.0)
  return out


def _model_summary(m: Any) -> dict[str, Any]:
  pos = getattr(m, "position", None)
  vel = getattr(m, "velocity", None)
  orient_rate = getattr(m, "orientationRate", None)
  return {
    "position_x": _take_nested_numeric(pos, "x", 20),
    "position_y": _take_nested_numeric(pos, "y", 20),
    "velocity_x": _take_nested_numeric(vel, "x", 20),
    "velocity_y": _take_nested_numeric(vel, "y", 20),
    "orientationRate_z": _take_nested_numeric(orient_rate, "z", 20),
  }


def _tail_swaglog(txt_path: Path) -> None:
  log_dir = Path("/data/log")
  candidates = [
    log_dir / "swaglog",
    log_dir / "swaglog.0000000000",
  ]
  for path in sorted(log_dir.glob("swaglog*"), reverse=True):
    if path.is_file():
      candidates.append(path)

  log_path = None
  for path in candidates:
    if path.exists() and path.is_file():
      log_path = path
      break

  with txt_path.open("a", encoding="utf-8") as out_f:
    out_f.write(f"# tail_source={str(log_path) if log_path else 'none'}\n")
    out_f.flush()

    if log_path is None:
      return

    try:
      with log_path.open("r", encoding="utf-8", errors="replace") as in_f:
        in_f.seek(0, os.SEEK_END)
        while RUNNING:
          line = in_f.readline()
          if not line:
            time.sleep(0.20)
            continue
          if "[XNOR_CRUISE_SYNC]" in line or "[XNOR_CRUISE_IDLE]" in line:
            out_f.write(line)
            out_f.flush()
    except Exception as e:
      out_f.write(f"# swaglog_tail_error={e}\n")
      out_f.flush()


def main() -> int:
  parser = argparse.ArgumentParser(description="Dump planner-origin watch data to /data as jsonl + txt.")
  parser.add_argument("duration_s", nargs="?", type=float, default=240.0)
  parser.add_argument("--hz", type=float, default=10.0)
  args = parser.parse_args()

  signal.signal(signal.SIGINT, _sig_handler)
  signal.signal(signal.SIGTERM, _sig_handler)

  hz = max(1.0, float(args.hz))
  period_s = 1.0 / hz
  duration_s = max(1.0, float(args.duration_s))

  stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
  jsonl_path = Path(f"/data/planner_origin_watch_{stamp}.jsonl")
  txt_path = Path(f"/data/planner_origin_watch_{stamp}.txt")

  topics = ["carState", "controlsState", "radarState", "longitudinalPlan", "modelV2", "mapdOut"]
  sm = messaging.SubMaster(topics)

  tail_thread = threading.Thread(target=_tail_swaglog, args=(txt_path,), daemon=True)
  tail_thread.start()

  start_ms = _now_ms()
  end_ms = start_ms + int(duration_s * 1000.0)
  sample_count = 0

  with jsonl_path.open("w", encoding="utf-8") as out_f:
    header = {
      "type": "planner_origin_watch_header",
      "wall_time": _wall_iso(),
      "duration_s": duration_s,
      "hz": hz,
      "topics": topics,
      "jsonl_path": str(jsonl_path),
      "txt_path": str(txt_path),
    }
    out_f.write(json.dumps(header, separators=(",", ":")) + "\n")
    out_f.flush()

    with txt_path.open("a", encoding="utf-8") as txt_f:
      txt_f.write("planner-origin watch\n")
      txt_f.write(f"duration_s={duration_s}\n")
      txt_f.write(f"hz={hz}\n")
      txt_f.write(f"output={jsonl_path}\n")
      txt_f.write(f"topics={','.join(topics)} + XNOR_CRUISE_SYNC tail\n")
      txt_f.flush()

    next_tick = time.monotonic()
    while RUNNING and _now_ms() < end_ms:
      sm.update(0)

      record = {
        "type": "planner_origin_watch_sample",
        "wall_time": _wall_iso(),
        "mono_ms": _now_ms(),
        "updated": {topic: bool(sm.updated.get(topic, False)) for topic in topics},
        "valid": {topic: bool(sm.valid.get(topic, False)) for topic in topics},
        "logMonoTime": {topic: int(sm.logMonoTime.get(topic, 0) or 0) for topic in topics},
      }

      try:
        record["carState"] = _car_state_summary(sm["carState"])
      except Exception as e:
        record["carState_error"] = str(e)

      try:
        record["controlsState"] = _controls_state_summary(sm["controlsState"])
      except Exception as e:
        record["controlsState_error"] = str(e)

      try:
        rs = sm["radarState"]
        record["radarState"] = {
          "leadOne": _lead_summary(getattr(rs, "leadOne", None)),
          "leadTwo": _lead_summary(getattr(rs, "leadTwo", None)),
        }
      except Exception as e:
        record["radarState_error"] = str(e)

      try:
        record["longitudinalPlan"] = _plan_summary(sm["longitudinalPlan"])
      except Exception as e:
        record["longitudinalPlan_error"] = str(e)

      try:
        record["modelV2"] = _model_summary(sm["modelV2"])
      except Exception as e:
        record["modelV2_error"] = str(e)

      try:
        record["mapdOut"] = _mapd_summary(sm["mapdOut"])
      except Exception as e:
        record["mapdOut_error"] = str(e)

      out_f.write(json.dumps(record, separators=(",", ":")) + "\n")
      out_f.flush()
      sample_count += 1

      next_tick += period_s
      sleep_s = next_tick - time.monotonic()
      if sleep_s > 0:
        time.sleep(sleep_s)

  with txt_path.open("a", encoding="utf-8") as txt_f:
    txt_f.write(f"# samples={sample_count}\n")
    txt_f.write(f"# finished_wall_time={_wall_iso()}\n")
    txt_f.flush()

  print(json.dumps({
    "jsonl": str(jsonl_path),
    "txt": str(txt_path),
    "samples": sample_count,
  }))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
