#!/usr/bin/env python3
from __future__ import annotations

import glob
import json
import os
import time
from collections import Counter, deque
from typing import Any, Optional

import cereal.messaging as messaging

ADDR_STW = 0x045

def get_bus(item: Any) -> int:
  return int(getattr(item, "bus", getattr(item, "src", 0)))

def latest_swaglog() -> Optional[str]:
  files = glob.glob("/data/log/swaglog.*")
  if not files:
    return None
  files.sort(key=lambda p: os.path.getmtime(p))
  return files[-1]

def main() -> None:
  dur = 30.0

  tx_sock = messaging.sub_sock("sendcan", conflate=False, timeout=1000)
  can_sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  cc_sock = messaging.sub_sock("carControl", conflate=True, timeout=1000)
  cs_sock = messaging.sub_sock("carState", conflate=True, timeout=1000)
  ctl_sock = messaging.sub_sock("controlsState", conflate=True, timeout=1000)

  log_path = latest_swaglog()
  print("swaglog:", log_path)

  log_f = None
  if log_path:
    log_f = open(log_path, "r", encoding="utf-8", errors="ignore")
    log_f.seek(0, os.SEEK_END)

  tx_counts = Counter()
  stw_tx_samples = deque(maxlen=10)

  rx_counts = Counter()
  stw_rx_src = Counter()

  last_hb = -1
  start = time.monotonic()

  print("\nINSTRUCTIONS:")
  print(" - Run for 30s while you drive >18mph and attempt to enable/auto-engage.")
  print(" - This will print any XNOR_CRUISE_SYNC log lines and any TX 0x045 frames.\n")

  while time.monotonic() - start < dur:
    now = time.monotonic()
    hb = int(now - start)
    if hb != last_hb:
      last_hb = hb
      print(f"t={hb:2d}s tx_stw={tx_counts[ADDR_STW]} rx_stw={rx_counts[ADDR_STW]}")

      # snapshots (schema-safe)
      cc = messaging.recv_sock(cc_sock)
      if cc is not None:
        try:
          c = cc.carControl
          bits = []
          for k in ("enabled", "latActive", "longActive"):
            if hasattr(c, k):
              bits.append(f"{k}={getattr(c,k)}")
          print("  carControl:", " ".join(bits) if bits else "(schema differs)")
        except Exception:
          pass

      cs = messaging.recv_sock(cs_sock)
      if cs is not None:
        try:
          s = cs.carState
          bits = []
          if hasattr(s, "vEgo"):
            bits.append(f"vEgo={float(s.vEgo):.2f}")
          if hasattr(s, "cruiseState"):
            bits.append(f"cruise_enabled={bool(s.cruiseState.enabled)}")
            bits.append(f"cruise_avail={bool(s.cruiseState.available)}")
            bits.append(f"cruise_speed={float(s.cruiseState.speed):.2f}")
          print("  carState:", " ".join(bits))
        except Exception:
          pass

      ctl = messaging.recv_sock(ctl_sock)
      if ctl is not None:
        try:
          st = ctl.controlsState
          # don't assume `.enabled` exists
          ks = []
          for k in ("state", "alertText1", "alertText2", "alertStatus"):
            if hasattr(st, k):
              ks.append(f"{k}={getattr(st,k)}")
          if ks:
            print("  controlsState:", " ".join(ks))
        except Exception:
          pass

    # tail swaglog for XNOR_CRUISE_SYNC
    if log_f is not None:
      where = log_f.tell()
      line = log_f.readline()
      if not line:
        log_f.seek(where)
      else:
        if "XNOR_CRUISE_SYNC" in line:
          try:
            j = json.loads(line)
            msg = j.get("msg$s") or j.get("msg") or ""
            print("  LOG:", msg)
          except Exception:
            print("  LOG:", line.strip())

    # sendcan TX
    tx = messaging.recv_sock(tx_sock)
    if tx is not None:
      for m in tx.sendcan:
        a = int(m.address)
        tx_counts[a] += 1
        if a == ADDR_STW:
          stw_tx_samples.append((get_bus(m), bytes(m.dat).hex()))

    # can RX (just for STW src accounting)
    rx = messaging.recv_sock(can_sock)
    if rx is not None:
      for m in rx.can:
        a = int(m.address)
        if a != ADDR_STW:
          continue
        rx_counts[a] += 1
        stw_rx_src[int(m.src)] += 1

  print("\n=== SUMMARY ===")
  print("TX STW frames:", tx_counts[ADDR_STW])
  if stw_tx_samples:
    print("TX STW samples (tx_bus/src, raw):")
    for b, raw in stw_tx_samples:
      print(f"  {b:3d} {raw}")
  else:
    print("TX STW samples: (none)")

  print("RX STW by src:", dict(stw_rx_src))

if __name__ == "__main__":
  main()
