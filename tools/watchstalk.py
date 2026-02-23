#!/usr/bin/env python3
from __future__ import annotations

import time
from collections import Counter, defaultdict
import cereal.messaging as messaging

ADDR = 0x659

def get_bus(item) -> int:
  return int(getattr(item, "bus", getattr(item, "src", 0)))

def main() -> None:
  tx = messaging.sub_sock("sendcan", conflate=False, timeout=1000)
  rx = messaging.sub_sock("can", conflate=False, timeout=1000)

  dur = 12.0
  start = time.monotonic()
  last = -1

  tx_b5 = Counter()        # (bus, b5) -> count
  rx_b5 = Counter()        # (src, b5) -> count

  print("Run 12s. During run: pull stalk MAIN a few times (and release).")

  while time.monotonic() - start < dur:
    now = time.monotonic()
    t = int(now - start)
    if t != last:
      last = t
      print(f"t={t:2d}s tx_samples={sum(tx_b5.values())} rx_samples={sum(rx_b5.values())}")

    m = messaging.recv_sock(tx)
    if m is not None:
      for it in m.sendcan:
        if int(it.address) != ADDR:
          continue
        dat = bytes(it.dat)
        b5 = dat[5] if len(dat) > 5 else 0
        tx_b5[(get_bus(it), b5)] += 1

    m = messaging.recv_sock(rx)
    if m is not None:
      for it in m.can:
        if int(it.address) != ADDR:
          continue
        dat = bytes(it.dat)
        b5 = dat[5] if len(dat) > 5 else 0
        rx_b5[(int(it.src), b5)] += 1

  print("\n=== TX 0x659 byte5 distribution (sendcan) ===")
  for (bus, b5), c in tx_b5.most_common():
    print(f"tx_bus/src={bus:3d} b5=0x{b5:02x} count={c}")

  print("\n=== RX 0x659 byte5 distribution (can) ===")
  for (src, b5), c in rx_b5.most_common():
    print(f"rx_src={src:3d} b5=0x{b5:02x} count={c}")

  print("\nInterpretation:")
  print(" - If you never see b5 with bit 0x40 set, enable-latch can never happen.")
  print(" - If TX shows only one bus, panda1 will never latch, causing controlsMismatch.")

if __name__ == "__main__":
  main()
