#!/usr/bin/env python3
# /data/openpilot/tools/can_error_diag.py
"""
Diagnose "CAN error: check connections" on XNOR/Tesla.

Prints:
- Process presence (card/pandad/controlsd)
- managerState card status (if available)
- pandaStates: canValid, safetyRxChecksInvalid, controlsAllowed, ignition, faults, canState bus-off/errors (if exposed)
- carState presence + vEgo + canValid (if exposed)
- controlsState alert fields (schema-safe)
- can RX frames/sec by src
- tail of latest swaglog for Traceback/crash/can errors
"""

from __future__ import annotations

import glob
import json
import os
import subprocess
import time
from collections import Counter, defaultdict
from typing import Any, Optional

import cereal.messaging as messaging

try:
  from common.params import Params
except Exception:
  from openpilot.common.params import Params  # type: ignore


def _pgrep(pattern: str) -> list[str]:
  try:
    out = subprocess.check_output(["pgrep", "-fa", pattern], text=True).strip()
    return out.splitlines() if out else []
  except Exception:
    return []


def _latest_swaglog() -> Optional[str]:
  files = glob.glob("/data/log/swaglog.*")
  if not files:
    return None
  files.sort(key=lambda p: os.path.getmtime(p))
  return files[-1]


def _tail_find(path: str, needles: tuple[str, ...], max_lines: int = 2500, max_print: int = 80) -> None:
  try:
    lines = open(path, "r", encoding="utf-8", errors="ignore").read().splitlines()
  except Exception as e:
    print("Failed reading swaglog:", e)
    return
  tail = lines[-max_lines:] if len(lines) > max_lines else lines
  hits = [ln for ln in tail if any(n in ln for n in needles)]
  if not hits:
    print("No obvious traceback/crash lines in latest swaglog tail.")
    return
  print(f"\n=== Latest swaglog hits in {path} (last {min(len(hits), max_print)} matches) ===")
  for ln in hits[-max_print:]:
    print(ln)


def _fields(obj: Any) -> list[str]:
  try:
    return list(obj.schema.fields.keys())
  except Exception:
    return []


def _safe_get(obj: Any, name: str) -> Optional[Any]:
  return getattr(obj, name) if hasattr(obj, name) else None


def main() -> None:
  print("=== Processes ===")
  for name, pat in [
    ("manager", "manager.py"),
    ("controlsd", "selfdrive.controls.controlsd"),
    ("pandad", "selfdrive.pandad.pandad"),
    ("card", "selfdrive.car.card"),
  ]:
    hits = _pgrep(pat)
    print(f"{name:10s}:", "YES" if hits else "NO")
    for ln in hits[:2]:
      print(" ", ln)

  print("\n=== Params ===")
  try:
    print("IsOnroad:", bool(Params().get_bool("IsOnroad")))
  except Exception:
    print("IsOnroad: (unknown)")
  try:
    print("CarParams set:", Params().get("CarParams") is not None)
  except Exception:
    print("CarParams set: (unknown)")

  # managerState (card status)
  print("\n=== managerState (card) ===")
  try:
    ms_sock = messaging.sub_sock("managerState", conflate=True, timeout=500)
    ms = messaging.recv_sock(ms_sock)
  except Exception:
    ms = None

  if ms is None:
    print("managerState: not available")
  else:
    try:
      st = ms.managerState
      found = False
      for p in st.processes:
        if getattr(p, "name", "") == "card":
          found = True
          keys = ["running", "shouldBeRunning", "pid", "exitCode", "sig", "exitTs"]
          data = {k: getattr(p, k) for k in keys if hasattr(p, k)}
          print("card:", data)
      if not found:
        print("card: not found in managerState.processes")
    except Exception as e:
      print("managerState parse failed:", e)

  # Subscribe to services for a few seconds
  panda_sock = messaging.sub_sock("pandaStates", conflate=True, timeout=1000)
  can_sock = messaging.sub_sock("can", conflate=False, timeout=1000)

  # controlsState + carState may have schema differences; be defensive
  cs_sock = messaging.sub_sock("carState", conflate=True, timeout=1000)
  ctl_sock = messaging.sub_sock("controlsState", conflate=True, timeout=1000)

  print("\n=== Live sampling (5s) ===")
  start = time.monotonic()
  dur = 5.0

  # can frames/sec by src
  can_by_src = Counter()

  last_panda = None
  last_car = None
  last_ctl = None

  while time.monotonic() - start < dur:
    msg = messaging.recv_sock(can_sock)
    if msg is not None:
      for m in msg.can:
        can_by_src[int(m.src)] += 1

    ps = messaging.recv_sock(panda_sock)
    if ps is not None:
      last_panda = ps

    cs = messaging.recv_sock(cs_sock)
    if cs is not None:
      last_car = cs

    ctl = messaging.recv_sock(ctl_sock)
    if ctl is not None:
      last_ctl = ctl

  # Print can rate
  total_frames = sum(can_by_src.values())
  print(f"can frames in {dur:.1f}s: {total_frames} ({total_frames/dur:.1f} fps), srcs={sorted(can_by_src.keys())}")
  for src, cnt in can_by_src.most_common(10):
    print(f"  rx_src={src:3d} fps={cnt/dur:.1f}")

  # Print pandaStates
  print("\n=== pandaStates ===")
  if last_panda is None:
    print("NO pandaStates messages")
  else:
    try:
      pds = list(last_panda.pandaStates)
      print(f"pandas={len(pds)}")
      for i, p in enumerate(pds):
        out = [f"panda[{i}]"]
        for k in ("pandaType", "serial", "safetyModel", "safetyParam", "controlsAllowed",
                  "canValid", "safetyRxChecksInvalid", "ignitionCan", "ignitionLine", "faultStatus",
                  "safetyTxBlocked", "safetyRxInvalid", "heartbeatLost"):
          v = _safe_get(p, k)
          if v is not None:
            out.append(f"{k}={v}")
        print("  " + " ".join(out))

        # canState details if exposed
        for k in ("canState0", "canState1", "canState2"):
          st = _safe_get(p, k)
          if st is None:
            continue
          # common fields: busOff, rxErrorCount, txErrorCount, rxCnt, txCnt
          keys = _fields(st)
          show = {}
          for kk in ("busOff", "rxErrorCount", "txErrorCount", "rxCnt", "txCnt", "errorWarning", "errorPassive"):
            if hasattr(st, kk):
              show[kk] = getattr(st, kk)
          if show:
            print(f"    {k}: {show}")
    except Exception as e:
      print("pandaStates parse failed:", e)

  # Print carState
  print("\n=== carState ===")
  if last_car is None:
    print("NO carState messages (card may be dead)")
  else:
    try:
      c = last_car.carState
      bits = []
      if hasattr(c, "vEgo"):
        bits.append(f"vEgo={float(c.vEgo):.2f}")
      if hasattr(c, "canValid"):
        bits.append(f"canValid={bool(c.canValid)}")
      if hasattr(c, "cruiseState"):
        bits.append(f"cruise_enabled={bool(c.cruiseState.enabled)}")
        bits.append(f"cruise_avail={bool(c.cruiseState.available)}")
      print("  " + " ".join(bits))
    except Exception as e:
      print("carState parse failed:", e)

  # Print controlsState
  print("\n=== controlsState (alerts) ===")
  if last_ctl is None:
    print("NO controlsState messages")
  else:
    try:
      st = last_ctl.controlsState
      # print whatever alert-like fields exist
      out = []
      for k in ("state", "alertText1", "alertText2", "alertStatus", "alertSize"):
        if hasattr(st, k):
          out.append(f"{k}={getattr(st,k)}")
      if out:
        print("  " + " ".join(out))
      else:
        print("  controlsState schema has no standard alert fields:", _fields(st))
    except Exception as e:
      print("controlsState parse failed:", e)

  # Tail swaglog for crashes/tracebacks/can
  path = _latest_swaglog()
  print("\n=== Latest swaglog scan ===")
  if path is None:
    print("No swaglog.* found")
  else:
    _tail_find(path, needles=("Traceback", "Process card", "crash", "can error", "CAN error", "Exception"), max_lines=3500)

  print("\nDONE. Paste this output.")

if __name__ == "__main__":
  main()
