#!/usr/bin/env python3
import argparse
import csv
import time
from pathlib import Path

import cereal.messaging as messaging

TAGS = ("XNOR_CC_DIAG", "XNOR_LONG_DIAG", "XNOR_CRUISE_SYNC")

def safe_get(obj, name, default=None):
  try:
    return getattr(obj, name)
  except Exception:
    return default

def main():
  parser = argparse.ArgumentParser(description="Capture XNOR diag log messages")
  parser.add_argument("--out", default="/data/media/0/realdata/xnor_gate_diag.csv")
  args = parser.parse_args()

  out_path = Path(args.out)
  out_path.parent.mkdir(parents=True, exist_ok=True)

  sm = messaging.SubMaster(["logMessage"], ignore_avg_freq=True)

  with out_path.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["ts_wall", "tag", "msg"])
    writer.writeheader()
    while True:
      sm.update(100)
      if not sm.updated.get("logMessage", False):
        continue
      lm = sm["logMessage"]
      msg = safe_get(lm, "msg", "") or ""
      for tag in TAGS:
        if tag in msg:
          writer.writerow({"ts_wall": f"{time.time():.3f}", "tag": tag, "msg": msg})
          f.flush()
          break

if __name__ == "__main__":
  main()
