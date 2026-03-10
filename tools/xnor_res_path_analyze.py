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
  parser = argparse.ArgumentParser(description="Analyze XNOR RES path probe CSV")
  parser.add_argument("csv_path")
  args = parser.parse_args()

  rows = []
  with open(args.csv_path, newline="") as fcsv:
    reader = csv.DictReader(fcsv)
    for row in reader:
      rows.append(row)

  # Focus on rows where automated speed-up would be expected and driver is not overriding.
  focus = []
  for r in rows:
    gas = i(r.get("gas_pressed", 0))
    brake = i(r.get("brake_pressed", 0))
    stock_enabled = i(r.get("stock_cruise_enabled", 0))
    stock_set_ms = f(r.get("stock_set_ms", 0.0))
    controls_v = f(r.get("controls_vcruise", 0.0))
    car_v = f(r.get("car_vcruise", 0.0))
    lp_last = f(r.get("lp_last", 0.0))
    demand_up = (max(controls_v, car_v) > stock_set_ms * 3.6 + 1.0) or (lp_last > stock_set_ms + 0.5)
    if stock_enabled and not gas and not brake and demand_up:
      focus.append(r)

  print(f"Focus rows (no override + speed-up demand): {len(focus)}")

  if not focus:
    print("No useful focus rows captured.")
    return 0

  counts = Counter()
  for r in focus:
    xnor_btn = i(r.get("xnor_btn", 0))
    sendcan_seen = i(r.get("sendcan_659_seen", 0))
    panda_allow = i(r.get("panda0_controls_allowed", 0))
    stock_set_ms = f(r.get("stock_set_ms", 0.0))
    controls_v = f(r.get("controls_vcruise", 0.0))
    car_v = f(r.get("car_vcruise", 0.0))
    lp_last = f(r.get("lp_last", 0.0))
    speedup_demand = (max(controls_v, car_v) > stock_set_ms * 3.6 + 1.0) or (lp_last > stock_set_ms + 0.5)

    if speedup_demand and xnor_btn == 0:
      counts["no_xnor_res_decision"] += 1
    if xnor_btn != 0 and not sendcan_seen:
      counts["xnor_decision_no_sendcan"] += 1
    if sendcan_seen:
      counts["sendcan_seen"] += 1
    if panda_allow:
      counts["panda_controls_allowed"] += 1

  print("Counts:")
  for k, v in counts.most_common():
    print(f"  {k}: {v}")

  print("\nInterpretation:")
  if counts["no_xnor_res_decision"] > 0 and counts["xnor_decision_no_sendcan"] == 0:
    print("  Main issue appears upstream of sendcan: LONG/ACC is not deciding to raise speed.")
  if counts["xnor_decision_no_sendcan"] > 0:
    print("  LONG/ACC is deciding something, but 0x659 sendcan is not appearing in the same sample windows.")
  if counts["sendcan_seen"] > 0 and counts["no_xnor_res_decision"] == 0:
    print("  0x659 sendcan is present during speed-up demand; remaining issue is likely readback/registration.")
  return 0


if __name__ == "__main__":
  main()
