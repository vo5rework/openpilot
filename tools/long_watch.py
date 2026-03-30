#!/usr/bin/env python3
from __future__ import annotations

import glob
import json
import os
import signal
import sys
import time
from typing import Any

import cereal.messaging as messaging


def latest_swaglog() -> str | None:
  files = glob.glob("/data/log/swaglog.*")
  if not files:
    return None
  files.sort(key=os.path.getmtime)
  return files[-1]


def sget(obj: Any, path: str, default: Any = None) -> Any:
  cur = obj
  for part in path.split("."):
    if cur is None:
      return default
    try:
      cur = getattr(cur, part)
    except Exception:
      return default
  return cur if cur is not None else default


def lead_dict(lead: Any) -> dict[str, Any]:
  if lead is None:
    return {}
  out: dict[str, Any] = {}
  for key in ("status", "radar", "radarTrackId", "dRel", "vRel", "aRel", "yRel", "vLead", "aLeadK", "fcw"):
    try:
      val = getattr(lead, key)
      if isinstance(val, (bool, int, float, str)):
        out[key] = val
    except Exception:
      pass
  return out


def long_plan_dict(lp: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  if lp is None:
    return out
  for key in ("hasLead", "aTarget", "vTarget", "vTargetFuture", "jerkFactor"):
    try:
      val = getattr(lp, key)
      if isinstance(val, (bool, int, float, str)):
        out[key] = val
    except Exception:
      pass

  for arr_key in ("speeds", "accels", "jerks"):
    try:
      arr = list(getattr(lp, arr_key))
      out[arr_key] = [float(x) for x in arr[:6]]
    except Exception:
      pass

  try:
    leads = getattr(lp, "longitudinalPlanSource")
    if isinstance(leads, str):
      out["longitudinalPlanSource"] = leads
  except Exception:
    pass
  return out


def model_dict(model: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  if model is None:
    return out
  try:
    act = getattr(model, "action")
    for key in ("desiredCurvature", "desiredCurvatureRate"):
      try:
        out[key] = float(getattr(act, key))
      except Exception:
        pass
  except Exception:
    pass
  return out


def cs_dict(cs: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  if cs is None:
    return out
  keys = [
    "vEgo", "aEgo", "standstill", "gasPressed", "brakePressed", "steeringAngleDeg",
    "steeringPressed", "leftBlinker", "rightBlinker"
  ]
  for key in keys:
    try:
      val = getattr(cs, key)
      if isinstance(val, (bool, int, float, str)):
        out[key] = val
    except Exception:
      pass
  try:
    cruise = getattr(cs, "cruiseState")
    cruise_out = {}
    for key in ("enabled", "available", "standstill", "speed", "speedCluster"):
      try:
        val = getattr(cruise, key)
        if isinstance(val, (bool, int, float, str)):
          cruise_out[key] = val
      except Exception:
        pass
    out["cruiseState"] = cruise_out
  except Exception:
    pass
  return out


def cc_dict(cc: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  if cc is None:
    return out
  for key in ("enabled", "latActive", "longActive"):
    try:
      val = getattr(cc, key)
      if isinstance(val, (bool, int, float, str)):
        out[key] = val
    except Exception:
      pass
  try:
    actuators = getattr(cc, "actuators")
    act_out = {}
    for key in ("accel", "steeringAngleDeg"):
      try:
        act_out[key] = float(getattr(actuators, key))
      except Exception:
        pass
    out["actuators"] = act_out
  except Exception:
    pass
  return out


def parse_sync_line(line: str) -> dict[str, Any] | None:
  if "XNOR_CRUISE_SYNC" not in line:
    return None
  try:
    j = json.loads(line)
  except Exception:
    return {"raw": line.strip()}

  msg = j.get("msg$s") or j.get("msg") or ""
  out = {
    "created": j.get("created"),
    "filename": j.get("filename"),
    "lineno": j.get("lineno"),
    "msg": msg,
  }

  parts = {}
  for token in msg.replace("[XNOR_CRUISE_SYNC]", "").strip().split():
    if "=" in token:
      k, v = token.split("=", 1)
      parts[k] = v
  out["parts"] = parts
  return out


def main() -> None:
  dur_s = 180.0
  hz = 10.0
  if len(sys.argv) >= 2:
    dur_s = float(sys.argv[1])

  ts = time.strftime("%Y%m%d_%H%M%S")
  out_path = f"/data/long_watch_{ts}.jsonl"
  meta_path = f"/data/long_watch_{ts}.txt"

  with open(meta_path, "w", encoding="utf-8") as meta:
    meta.write("LONG watch\n")
    meta.write(f"duration_s={dur_s}\n")
    meta.write(f"output={out_path}\n")
    meta.write("topics=carState,carControl,controlsState,radarState,longitudinalPlan,modelV2 + XNOR_CRUISE_SYNC tail\n")

  log_path = latest_swaglog()
  log_f = None
  if log_path:
    log_f = open(log_path, "r", encoding="utf-8", errors="ignore")
    log_f.seek(0, os.SEEK_END)

  socks = {
    "carState": messaging.sub_sock("carState", conflate=True, timeout=1000),
    "carControl": messaging.sub_sock("carControl", conflate=True, timeout=1000),
    "controlsState": messaging.sub_sock("controlsState", conflate=True, timeout=1000),
    "radarState": messaging.sub_sock("radarState", conflate=True, timeout=1000),
    "longitudinalPlan": messaging.sub_sock("longitudinalPlan", conflate=True, timeout=1000),
    "modelV2": messaging.sub_sock("modelV2", conflate=True, timeout=1000),
  }

  latest: dict[str, Any] = {}
  stop = False

  def _sig(_n: int, _f: Any) -> None:
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, _sig)
  signal.signal(signal.SIGTERM, _sig)

  period = 1.0 / hz
  start = time.monotonic()
  next_t = start

  with open(out_path, "w", encoding="utf-8") as out_f:
    while not stop and (time.monotonic() - start) < dur_s:
      for name, sock in socks.items():
        msg = messaging.recv_sock(sock)
        if msg is None:
          continue
        try:
          latest[name] = getattr(msg, name)
        except Exception:
          latest[name] = msg

      while log_f is not None:
        where = log_f.tell()
        line = log_f.readline()
        if not line:
          log_f.seek(where)
          break
        sync = parse_sync_line(line)
        if sync is not None:
          latest["XNOR_CRUISE_SYNC"] = sync

      now = time.monotonic()
      if now >= next_t:
        rec = {
          "mono": now - start,
          "carState": cs_dict(latest.get("carState")),
          "carControl": cc_dict(latest.get("carControl")),
          "controlsState": {
            "state": sget(latest.get("controlsState"), "state"),
            "enabled": sget(latest.get("controlsState"), "enabled"),
            "vCruise": sget(latest.get("controlsState"), "vCruise"),
            "vCruiseCluster": sget(latest.get("controlsState"), "vCruiseCluster"),
          },
          "radarLeadOne": lead_dict(sget(latest.get("radarState"), "leadOne")),
          "radarLeadTwo": lead_dict(sget(latest.get("radarState"), "leadTwo")),
          "longitudinalPlan": long_plan_dict(latest.get("longitudinalPlan")),
          "modelV2": model_dict(latest.get("modelV2")),
          "sync": latest.get("XNOR_CRUISE_SYNC"),
        }
        out_f.write(json.dumps(rec, separators=(",", ":"), ensure_ascii=False) + "\n")
        out_f.flush()
        next_t += period

      sleep_for = max(0.0, min(period / 3.0, next_t - time.monotonic()))
      time.sleep(sleep_for)

  print(out_path)
  print(meta_path)


if __name__ == "__main__":
  main()
