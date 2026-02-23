#!/usr/bin/env python3
"""Trace panda enable + TX distribution for Tesla legacy (fork-safe).

This answers:
- Which panda never reaches controlsAllowed=True (and why: ignitionCan/Line/canValid/safetyRxChecksInvalid)
- Which TX buses are used for safety-critical keepalives (0x659/0x488/0x27D) and STW (0x045)

Run (while trying to engage):
  python3 /data/openpilot/tools/panda_enable_trace.py --duration 20
"""

from __future__ import annotations

import argparse
import time
from collections import Counter, defaultdict

import cereal.messaging as messaging

WATCH = {
  0x659: "0x659",
  0x488: "0x488",
  0x27D: "0x27D",
  0x045: "0x045",
}

def get_bus(item) -> int:
  return int(getattr(item, "bus", getattr(item, "src", 0)))

def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--duration", type=float, default=20.0)
  ap.add_argument("--hz", type=float, default=10.0)
  args = ap.parse_args()

  ps_sock = messaging.sub_sock("pandaStates", conflate=True, timeout=1000)
  tx_sock = messaging.sub_sock("sendcan", conflate=False, timeout=1000)
  cs_sock = messaging.sub_sock("carState", conflate=True, timeout=1000)

  tx_counts = Counter()  # (addr, bus) -> count

  start = time.monotonic()
  tick = 1.0 / float(args.hz)
  next_t = start

  print("Tracing... try to engage during this window.")
  while time.monotonic() - start < float(args.duration):
    now = time.monotonic()

    # drain sendcan quickly
    tx = messaging.recv_sock(tx_sock)
    if tx is not None:
      for m in tx.sendcan:
        a = int(m.address)
        if a in WATCH:
          b = get_bus(m)
          tx_counts[(a, b)] += 1

    if now < next_t:
      continue
    next_t += tick

    # pandaStates snapshot
    ps = messaging.recv_sock(ps_sock)
    if ps is not None:
      try:
        pds = list(ps.pandaStates)
      except Exception:
        pds = []
      print(f"\n[t={now-start:5.2f}s] pandas={len(pds)}")
      for i, p in enumerate(pds):
        parts = [f"panda[{i}]"]
        for k in ("safetyModel","safetyParam","controlsAllowed","safetyRxChecksInvalid","canValid","ignitionCan","ignitionLine","faultStatus"):
          if hasattr(p, k):
            parts.append(f"{k}={getattr(p,k)}")
        print("  " + " ".join(parts))

    # carState snapshot (minimal)
    cs = messaging.recv_sock(cs_sock)
    if cs is not None:
      try:
        c = cs.carState
        v = float(getattr(c, "vEgo", 0.0))
        en = bool(c.cruiseState.enabled) if hasattr(c, "cruiseState") else False
        av = bool(c.cruiseState.available) if hasattr(c, "cruiseState") else False
        print(f"  carState: vEgo={v:.2f} cruise_enabled={en} cruise_avail={av}")
      except Exception:
        pass

    # TX distribution summary (rolling)
    if tx_counts:
      summ = defaultdict(int)
      for (a, b), cnt in tx_counts.items():
        summ[b] += cnt
      topb = ", ".join([f"bus{b}:{summ[b]}" for b in sorted(summ)])
      print("  sendcan buses (watched addrs):", topb)
      for a in sorted(WATCH):
        byb = {b: tx_counts.get((a,b),0) for b in sorted(summ)}
        if any(v>0 for v in byb.values()):
          print(f"   addr={WATCH[a]} per_bus={byb}")

  print("\nDONE.")

if __name__ == "__main__":
  main()
