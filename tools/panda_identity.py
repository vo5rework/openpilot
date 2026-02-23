#!/usr/bin/env python3
from __future__ import annotations

import time
import cereal.messaging as messaging

def main() -> None:
  sock = messaging.sub_sock("pandaStates", conflate=True, timeout=200)

  end = time.monotonic() + 5.0
  msg = None
  while time.monotonic() < end and msg is None:
    msg = messaging.recv_sock(sock)

  if msg is None:
    print("NO pandaStates messages in 5s")
    return

  try:
    pds = list(msg.pandaStates)
  except Exception as e:
    print("pandaStates parse failed:", e)
    return

  print(f"pandas={len(pds)}")
  for i, p in enumerate(pds):
    fields = list(p.schema.fields.keys())
    print(f"panda[{i}] fields={fields}")

    keys = [
      "serial", "pandaType", "hwType",
      "safetyModel", "safetyParam", "alternativeExperience",
      "controlsAllowed", "safetyRxChecksInvalid",
      "canValid", "ignitionCan", "ignitionLine", "faultStatus",
    ]
    out = [f"panda[{i}]"]
    for k in keys:
      if hasattr(p, k):
        out.append(f"{k}={getattr(p,k)}")
    print("  " + " ".join(out))

if __name__ == "__main__":
  main()
