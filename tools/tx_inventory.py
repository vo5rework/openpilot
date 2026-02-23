#!/usr/bin/env python3
"""
TX inventory for Tesla/XNOR: all sendcan addresses, tx bus/src distribution, Hz, samples.

Fork-safe:
- sendcan items may have `.bus` or `.src`; we treat bus as getattr(item,'bus',item.src)

Run:
  python3 /data/openpilot/tools/tx_inventory.py --duration 10
  python3 /data/openpilot/tools/tx_inventory.py --duration 10 --top 30
  python3 /data/openpilot/tools/tx_inventory.py --duration 10 --addr 0x045 0x488 0x27d 0x659
"""

from __future__ import annotations

import argparse
import time
from collections import Counter, defaultdict, deque
from typing import Optional, List

import cereal.messaging as messaging


def _get_bus(item) -> int:
  return int(getattr(item, "bus", getattr(item, "src", 0)))


def _parse_addrs(xs: Optional[List[str]]) -> Optional[set[int]]:
  if not xs:
    return None
  out: set[int] = set()
  for x in xs:
    x = x.strip().lower()
    out.add(int(x, 16) if x.startswith("0x") else int(x))
  return out


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--duration", type=float, default=10.0)
  ap.add_argument("--top", type=int, default=25)
  ap.add_argument("--addr", nargs="*", default=None, help="optional address filter list (hex)")
  args = ap.parse_args()

  addr_filter = _parse_addrs(args.addr)

  try:
    sock = messaging.sub_sock("sendcan", conflate=False, timeout=1000)
  except Exception as e:
    print("Failed to subscribe to sendcan:", e)
    return

  tx_msgs = 0
  tx_frames = 0

  per_addr = Counter()                # addr -> frames
  per_addr_bus = Counter()            # (addr, bus) -> frames
  samples = defaultdict(lambda: deque(maxlen=3))  # (addr,bus)-> rawhex

  start = time.monotonic()
  end = start + float(args.duration)

  while time.monotonic() < end:
    msg = messaging.recv_sock(sock)
    if msg is None:
      continue
    tx_msgs += 1
    frames = list(msg.sendcan)
    tx_frames += len(frames)

    for m in frames:
      addr = int(m.address)
      if addr_filter is not None and addr not in addr_filter:
        continue
      bus = _get_bus(m)
      dat = bytes(m.dat).hex()

      per_addr[addr] += 1
      per_addr_bus[(addr, bus)] += 1
      samples[(addr, bus)].append(dat)

  dur = max(time.monotonic() - start, 1e-6)

  print(f"\n=== TX INVENTORY (duration={dur:.2f}s) ===")
  print(f"sendcan_msgs={tx_msgs} sendcan_frames={tx_frames}")
  if tx_msgs == 0:
    print("No sendcan messages seen. Is card running?")
    return
  if tx_frames == 0:
    print("sendcan messages were published but contained empty lists.")
    return

  addrs_sorted = [a for a, _ in per_addr.most_common(args.top)]
  for addr in addrs_sorted:
    total = per_addr[addr]
    print(f"\naddr=0x{addr:03x} total={total:6d} ~{(total/dur):6.1f}Hz")
    buses = sorted({b for (a, b), c in per_addr_bus.items() if a == addr and c > 0})
    for b in buses:
      c = per_addr_bus[(addr, b)]
      samp = samples[(addr, b)][-1] if samples[(addr, b)] else ""
      print(f"  tx_bus/src={b:3d} count={c:6d} ~{(c/dur):6.1f}Hz last={samp}")

  print("\nDONE.")


if __name__ == "__main__":
  main()
