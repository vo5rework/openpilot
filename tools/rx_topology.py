#!/usr/bin/env python3
"""
RX topology for Tesla/XNOR: addresses by rx_src, rate (Hz), and mirroring detection.

What it does:
- Subscribes to `can`
- Counts frames per (addr, rx_src)
- Estimates Hz per addr and per src
- Detects mirroring: byte-identical payloads across different rx_src within a short window

Run:
  python3 /data/openpilot/tools/rx_topology.py --duration 10
  python3 /data/openpilot/tools/rx_topology.py --duration 20 --top 40
  python3 /data/openpilot/tools/rx_topology.py --duration 15 --addr 0x045 0x368 0x2f8

Notes:
- Mirroring is detected by comparing payload equality within --mirror-window-ms.
"""

from __future__ import annotations

import argparse
import time
from collections import Counter, defaultdict
from typing import Dict, Tuple, List, Optional

import cereal.messaging as messaging


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
  ap.add_argument("--duration", type=float, default=10.0, help="seconds")
  ap.add_argument("--top", type=int, default=30, help="show top N addrs by total frames")
  ap.add_argument("--addr", nargs="*", default=None, help="optional address filter list (hex)")
  ap.add_argument("--mirror-window-ms", type=float, default=25.0, help="mirror match window (ms)")
  ap.add_argument("--conflate", action="store_true", help="use conflate for can sock (lower CPU)")
  args = ap.parse_args()

  addr_filter = _parse_addrs(args.addr)
  mirror_window_s = float(args.mirror_window_ms) / 1000.0

  sock = messaging.sub_sock("can", conflate=bool(args.conflate), timeout=1000)

  # Counts
  per_addr = Counter()               # addr -> total frames
  per_addr_src = Counter()           # (addr, src) -> frames

  # Last payload per (addr, src) for mirror detection: (t, bytes)
  last_by_addr: Dict[int, Dict[int, Tuple[float, bytes]]] = defaultdict(dict)
  # Mirror matches per (addr, srcA, srcB)
  mirror_hits = Counter()            # (addr, a, b) -> matches

  start = time.monotonic()
  end = start + float(args.duration)

  while time.monotonic() < end:
    msg = messaging.recv_sock(sock)
    if msg is None:
      continue
    now = time.monotonic()

    for m in msg.can:
      addr = int(m.address)
      if addr_filter is not None and addr not in addr_filter:
        continue
      src = int(m.src)
      dat = bytes(m.dat)

      per_addr[addr] += 1
      per_addr_src[(addr, src)] += 1

      # mirror detection: compare against other src last payloads within window
      others = last_by_addr[addr]
      for osrc, (t_prev, d_prev) in others.items():
        if osrc == src:
          continue
        if now - t_prev > mirror_window_s:
          continue
        if d_prev == dat:
          a, b = (src, osrc) if src < osrc else (osrc, src)
          mirror_hits[(addr, a, b)] += 1

      others[src] = (now, dat)

  dur = max(time.monotonic() - start, 1e-6)

  # Choose which addrs to print
  addrs_sorted = [a for a, _ in per_addr.most_common(args.top)]

  print(f"\n=== RX TOPOLOGY (duration={dur:.2f}s, mirror_window={args.mirror_window_ms:.1f}ms) ===")
  if addr_filter is not None:
    print(f"Filtered addrs: {[hex(a) for a in sorted(addr_filter)]}")

  for addr in addrs_sorted:
    total = per_addr[addr]
    hz_total = total / dur
    srcs = sorted({s for (a, s), c in per_addr_src.items() if a == addr and c > 0})

    # per-src line
    parts = [f"addr=0x{addr:03x} total={total:6d} ~{hz_total:6.1f}Hz srcs={srcs}"]
    print("\n" + " ".join(parts))

    for s in srcs:
      c = per_addr_src[(addr, s)]
      print(f"  rx_src={s:3d} count={c:6d} ~{(c/dur):6.1f}Hz")

    # mirror summary (top pairs)
    pairs = [(k, v) for k, v in mirror_hits.items() if k[0] == addr]
    if pairs:
      pairs.sort(key=lambda kv: kv[1], reverse=True)
      print("  mirror_pairs (byte-identical within window):")
      for (a, s1, s2), hits in pairs[:6]:
        denom = min(per_addr_src[(addr, s1)], per_addr_src[(addr, s2)])
        ratio = (hits / denom) if denom > 0 else 0.0
        print(f"    {s1:3d}<->{s2:3d} hits={hits:6d} ratio={ratio:5.2f}")

  print("\nDONE.")


if __name__ == "__main__":
  main()
