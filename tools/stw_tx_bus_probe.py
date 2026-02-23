#!/usr/bin/env python3
from __future__ import annotations

import time
from collections import Counter, defaultdict
from typing import Any, Dict, Tuple, List, Optional

import cereal.messaging as messaging

from selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp  # type: ignore

ADDR_STW = 0x045

def get_bus(item: Any) -> int:
  return int(getattr(item, "bus", getattr(item, "src", 0)))

def recv_stw_idle(can_sock, timeout_s=3.0) -> Optional[bytes]:
  end = time.monotonic() + timeout_s
  while time.monotonic() < end:
    msg = messaging.recv_sock(can_sock)
    if msg is None:
      continue
    for m in msg.can:
      if int(m.address) != ADDR_STW:
        continue
      dat = bytes(m.dat)
      # idle signature: first byte 0x40 (SpdCtrl=0, Turn=0) in your logs
      if (dat[0] & 0x3F) == 0:
        return dat
  return None

def count_rx_stw(can_sock, seconds: float) -> Counter:
  end = time.monotonic() + seconds
  c = Counter()
  while time.monotonic() < end:
    msg = messaging.recv_sock(can_sock)
    if msg is None:
      continue
    for m in msg.can:
      if int(m.address) == ADDR_STW:
        c[int(m.src)] += 1
  return c

def send_probe(pm, dat: bytes, bus: int, seconds: float, hz: float) -> None:
  period = 1.0 / hz
  end = time.monotonic() + seconds
  while time.monotonic() < end:
    pm.send("sendcan", can_list_to_can_capnp([(ADDR_STW, dat, bus)], msgtype="sendcan", valid=True))
    time.sleep(period)

def main() -> None:
  can_sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  pm = messaging.PubMaster(["sendcan"])

  print("Capturing a live STW IDLE frame...")
  idle = recv_stw_idle(can_sock)
  if idle is None:
    print("FAILED: couldn't observe STW idle on can within 3s.")
    return
  print("Using idle raw:", idle.hex())

  baseline = count_rx_stw(can_sock, 2.0)
  print("Baseline RX counts (2s):", dict(baseline))

  # Candidate buses to probe. Start with ones already used in your fork.
  buses = [0, 4, 2, 6, 128, 130, 192, 196]

  results: Dict[int, Counter] = {}
  for b in buses:
    print(f"\nPROBE tx_bus/src={b} sending IDLE STW at 50Hz for 1.0s...")
    # measure before
    pre = count_rx_stw(can_sock, 0.5)
    send_probe(pm, idle, b, seconds=1.0, hz=50.0)
    post = count_rx_stw(can_sock, 0.5)

    # delta estimate
    delta = Counter()
    for k in set(pre) | set(post):
      delta[k] = post[k] - pre[k]
    results[b] = delta
    print("Delta RX (post-pre over 0.5s windows):", dict(delta))

  print("\n=== SUMMARY (tx_bus -> rx_src deltas) ===")
  for b, d in results.items():
    tops = sorted(d.items(), key=lambda kv: kv[1], reverse=True)
    tops = [(src, cnt) for src, cnt in tops if cnt > 0][:6]
    if tops:
      print(f"tx_bus/src={b:3d} -> {tops}")
    else:
      print(f"tx_bus/src={b:3d} -> (no measurable delta)")

  print("\nInterpretation:")
  print(" - You want a tx_bus that increases rx_src 0 and/or 130 (car's STW mirrors).")
  print(" - Once identified, CarController must send 0x045 on that tx_bus set.")
  print(" - This is routing truth; no guessing.")

if __name__ == "__main__":
  main()
