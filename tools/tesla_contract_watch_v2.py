#!/usr/bin/env python3
from __future__ import annotations
import time
from cereal import messaging

ADDR_STALK = 0x45
ADDR_FAKE  = 0x659

def lever_position(dat: bytes) -> int:
  return (dat[0] & 0x3F) if dat else -1

def parse_659(dat: bytes):
  b5 = dat[5] if len(dat) > 5 else 0
  return b5, bool(b5 & 0x80), bool(b5 & 0x20), bool(b5 & 0x02), bool(b5 & 0x01)

sm = messaging.SubMaster(["can", "sendcan", "pandaStates", "selfdriveState"], ignore_avg_freq=True)
seen_sendcan = set()
seen_659 = set()
last_summary = time.monotonic()

print("Watching: can(0x45), sendcan(0x659), pandaStates. Pull stalk. Ctrl+C.\n")

while True:
  sm.update(100)

  if sm.updated["can"]:
    for m in sm["can"]:
      if m.address == ADDR_STALK:
        dat = bytes(m.dat)
        lev = lever_position(dat)
        if lev != 0:
          print(f"[can 0x45] bus={m.src} dat={dat.hex()} lever={lev}")

  if sm.updated["sendcan"]:
    for m in sm["sendcan"]:
      seen_sendcan.add(m.src)
      if m.address == ADDR_FAKE:
        seen_659.add(m.src)
        b5, apd, ped, main, cancel = parse_659(bytes(m.dat))
        print(f"[sendcan 0x659] bus={m.src} b5=0x{b5:02x} ap_dis={apd} ped_en={ped} main_edge={main} cancel_edge={cancel}")

  if sm.updated["pandaStates"]:
    for i, ps in enumerate(sm["pandaStates"]):
      print(f"[pandaStates[{i}]] safetyModel={ps.safetyModel} safetyParam={ps.safetyParam} controlsAllowed={ps.controlsAllowed} faultStatus={ps.faultStatus}")

  now = time.monotonic()
  if now - last_summary >= 1.0:
    p0 = sm["pandaStates"][0] if len(sm["pandaStates"]) > 0 else None
    p1 = sm["pandaStates"][1] if len(sm["pandaStates"]) > 1 else None
    ptxt = ""
    if p0 and p1:
      ptxt = f"p0={p0.controlsAllowed}/{p0.safetyModel}:{p0.safetyParam}:{p0.faultStatus} p1={p1.controlsAllowed}/{p1.safetyModel}:{p1.safetyParam}:{p1.faultStatus}"
    print(f"[summary] sendcan_buses={sorted(seen_sendcan)} seen_0x659_buses={sorted(seen_659)} {ptxt}")
    last_summary = now
    seen_sendcan.clear()
    seen_659.clear()
