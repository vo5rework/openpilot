#!/usr/bin/env python3
# /data/openpilot/tools/rx_addr_presence.py
from __future__ import annotations
import time
from collections import defaultdict
import cereal.messaging as messaging

WATCH = [0x27D, 0x488, 0x659, 0x045, 0x368]  # EAC, steer, carrier, STW, DI

def main() -> None:
  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  seen = defaultdict(set)
  start = time.monotonic()
  dur = 5.0
  while time.monotonic() - start < dur:
    msg = messaging.recv_sock(sock)
    if msg is None:
      continue
    for m in msg.can:
      a = int(m.address)
      if a in WATCH:
        seen[a].add(int(m.src))

  for a in WATCH:
    print(f"addr=0x{a:03x} rx_srcs={sorted(seen[a])}")

if __name__ == "__main__":
  main()
