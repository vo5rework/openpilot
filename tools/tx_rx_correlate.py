#!/usr/bin/env python3
"""
Correlate sendcan TX bus/src -> can RX src by payload match (fork-safe).

This is the "no guessing" bus routing proof:
- For each address, store recent TX payloads with tx bus/src
- When RX sees same payload within a time window, attribute it to that tx bus

Run (while messages are actively being sent):
  python3 /data/openpilot/tools/tx_rx_correlate.py --duration 12 --addrs 0x045 0x488 0x27d 0x659 0x2bf

Options:
  --window-ms 200     correlation window
"""

from __future__ import annotations

import argparse
import time
from collections import defaultdict, deque
from typing import List

import cereal.messaging as messaging


def _get_bus(item) -> int:
  return int(getattr(item, "bus", getattr(item, "src", 0)))


def _parse_addrs(xs: List[str]) -> set[int]:
  out: set[int] = set()
  for x in xs:
    x = x.strip().lower()
    out.add(int(x, 16) if x.startswith("0x") else int(x))
  return out


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--duration", type=float, default=12.0)
  ap.add_argument("--window-ms", type=float, default=200.0)
  ap.add_argument("--addrs", nargs="*", default=["0x045", "0x488", "0x27d", "0x659"])
  args = ap.parse_args()

  addrs = _parse_addrs(args.addrs)
  window_s = float(args.window_ms) / 1000.0

  can_sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  tx_sock = messaging.sub_sock("sendcan", conflate=False, timeout=1000)

  # addr -> deque[(t, txbus, dat)]
  pending = defaultdict(lambda: deque(maxlen=4000))
  # (addr, txbus) -> rxsrc -> count
  mapping = defaultdict(lambda: defaultdict(int))
  # debug counts
  tx_seen = defaultdict(int)
  rx_seen = defaultdict(int)

  start = time.monotonic()
  end = start + float(args.duration)

  while time.monotonic() < end:
    tx = messaging.recv_sock(tx_sock)
    if tx is not None:
      now = time.monotonic()
      for m in tx.sendcan:
        a = int(m.address)
        if a not in addrs:
          continue
        b = _get_bus(m)
        dat = bytes(m.dat)
        pending[a].append((now, b, dat))
        tx_seen[a] += 1

    rx = messaging.recv_sock(can_sock)
    if rx is not None:
      now = time.monotonic()
      for m in rx.can:
        a = int(m.address)
        if a not in addrs:
          continue
        src = int(m.src)
        dat = bytes(m.dat)
        rx_seen[a] += 1

        # search newest-first within window
        for (t_tx, b, d_tx) in reversed(pending[a]):
          if now - t_tx > window_s:
            break
          if d_tx == dat:
            mapping[(a, b)][src] += 1
            break

  dur = max(time.monotonic() - start, 1e-6)
  print(f"\n=== TX->RX correlation (duration={dur:.2f}s, window={args.window_ms:.0f}ms) ===")
  print("Addrs:", [hex(a) for a in sorted(addrs)])
  print("\nTX frames seen by addr:", {hex(a): tx_seen[a] for a in sorted(tx_seen)})
  print("RX frames seen by addr:", {hex(a): rx_seen[a] for a in sorted(rx_seen)})

  print("\n=== Mapping (payload match) ===")
  if not mapping:
    print("(none) - ensure these addresses are actively being transmitted on sendcan.")
    return

  for (a, b), mp in sorted(mapping.items(), key=lambda x: (x[0][0], x[0][1])):
    top = sorted(mp.items(), key=lambda kv: kv[1], reverse=True)
    tops = ", ".join([f"rx_src={s}:{c}" for s, c in top[:10]])
    print(f"addr=0x{a:03x} tx_bus/src={b:3d} -> {tops}")

  print("\nDONE.")


if __name__ == "__main__":
  main()
