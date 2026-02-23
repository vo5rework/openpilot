#!/usr/bin/env python3
# /data/openpilot/tools/panda_identity.py
from __future__ import annotations
import time
import cereal.messaging as messaging

def main() -> None:
  sock = messaging.sub_sock("pandaStates", conflate=True, timeout=1000)
  msg = messaging.recv_sock(sock)
  if msg is None:
    print("No pandaStates message")
    return

  pds = list(msg.pandaStates)
  print(f"pandas={len(pds)}")
  for i, p in enumerate(pds):
    # Print everything that might exist without assuming schema
    keys = [
      "serial", "pandaType", "hwType", "safetyModel", "safetyParam",
      "controlsAllowed", "safetyRxChecksInvalid", "canValid",
      "ignitionCan", "ignitionLine", "faultStatus",
      "fanSpeedRpm", "voltage", "current",
    ]
    out = [f"panda[{i}]"]
    for k in keys:
      if hasattr(p, k):
        out.append(f"{k}={getattr(p,k)}")
    print("  " + " ".join(out))

if __name__ == "__main__":
  main()
