#!/usr/bin/env python3
import argparse
import csv
from collections import Counter


def f(x, default=0.0):
  try:
    return float(x)
  except Exception:
    return float(default)


def i(x, default=0):
  try:
    return int(x)
  except Exception:
    return int(default)


def main():
  parser = argparse.ArgumentParser(description="Analyze raw XNOR speed-up probe CSV")
  parser.add_argument("csv_path")
  args = parser.parse_args()

  rows = []
  with open(args.csv_path, newline="") as fcsv:
    reader = csv.DictReader(fcsv)
    for row in reader:
      rows.append(row)

  no_override = [r for r in rows if i(r.get("gas_pressed", 0)) == 0 and i(r.get("brake_pressed", 0)) == 0]
  print(f"No-override rows: {len(no_override)}")

  if not no_override:
    print("No useful non-override rows captured.")
    return 0

  counts = Counter()
  btn_counts = Counter()
  reason_counts = Counter()
  src_counts = Counter()

  last_set = None
  step_up_rows = 0
  step_down_rows = 0

  for r in no_override:
    xnor_recent = i(r.get("xnor_log_recent", 0))
    xnor_btn = i(r.get("xnor_btn", 0))
    sendcan_seen = i(r.get("sendcan_659_seen", 0))
    panda_allow = i(r.get("panda0_controls_allowed", 0))
    stock_set_ms = f(r.get("stock_set_ms", 0.0))

    if xnor_recent:
      counts["xnor_log_recent"] += 1
      btn_counts[xnor_btn] += 1
      reason_counts[r.get("xnor_reason", "")] += 1
      src_counts[r.get("xnor_src", "")] += 1
    if sendcan_seen:
      counts["sendcan_659_seen"] += 1
    if panda_allow:
      counts["panda_controls_allowed"] += 1

    if last_set is not None:
      if stock_set_ms > last_set + 0.05:
        step_up_rows += 1
      elif stock_set_ms < last_set - 0.05:
        step_down_rows += 1
    last_set = stock_set_ms

  print("Counts:")
  for k, v in counts.most_common():
    print(f"  {k}: {v}")

  print("Set-speed movement rows:")
  print(f"  step_up_rows: {step_up_rows}")
  print(f"  step_down_rows: {step_down_rows}")

  print("Top xnor_btn values:")
  for k, v in btn_counts.most_common(10):
    print(f"  {k}: {v}")

  print("Top reasons:")
  for k, v in reason_counts.most_common(10):
    print(f"  {k}: {v}")

  print("Top src:")
  for k, v in src_counts.most_common(10):
    print(f"  {k}: {v}")

  print("\nInterpretation:")
  if counts["xnor_log_recent"] == 0:
    print("  LONG/ACC is not emitting recent XNOR_CRUISE_SYNC decisions in the captured non-override rows.")
  elif counts["xnor_log_recent"] > 0 and counts["sendcan_659_seen"] == 0:
    print("  LONG/ACC is logging decisions, but 0x659 sendcan was not observed in the same sample windows.")
  elif counts["sendcan_659_seen"] > 0 and step_up_rows == 0 and step_down_rows > 0:
    print("  Commands are being emitted but observed set-speed still only steps down.")
  elif counts["sendcan_659_seen"] > 0 and step_up_rows == 0:
    print("  Commands are being emitted, but no set-speed increases were observed.")
  else:
    print("  Use btn/reason/src patterns plus step_up/step_down counts to locate the block.")

  return 0


if __name__ == "__main__":
  main()
