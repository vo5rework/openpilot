#!/usr/bin/env python3
import argparse
import csv
import re
from collections import Counter

GATE_RE = re.compile(r"gate=([^\s]+)")
REASON_RE = re.compile(r"reason=([^\s]+)")
SRC_RE = re.compile(r"src=([^\s]+)")
BTN_RE = re.compile(r"btn=(\d+)")

def main():
  parser = argparse.ArgumentParser(description="Analyze XNOR diag log CSV")
  parser.add_argument("csv_path")
  args = parser.parse_args()

  rows = list(csv.DictReader(open(args.csv_path, newline="")))
  print(f"Rows: {len(rows)}")

  tag_counts = Counter(r.get("tag", "") for r in rows)
  print("Tags:")
  for k,v in tag_counts.most_common():
    print(f"  {k}: {v}")

  gate_counts = Counter()
  reason_counts = Counter()
  src_counts = Counter()
  btn_counts = Counter()

  for r in rows:
    msg = r.get("msg","")
    m = GATE_RE.search(msg)
    if m:
      gate_counts[m.group(1)] += 1
    m = REASON_RE.search(msg)
    if m:
      reason_counts[m.group(1)] += 1
    m = SRC_RE.search(msg)
    if m:
      src_counts[m.group(1)] += 1
    m = BTN_RE.search(msg)
    if m:
      btn_counts[m.group(1)] += 1

  print("Gates:")
  for k,v in gate_counts.most_common():
    print(f"  {k}: {v}")

  print("Reasons:")
  for k,v in reason_counts.most_common():
    print(f"  {k}: {v}")

  print("Src:")
  for k,v in src_counts.most_common():
    print(f"  {k}: {v}")

  print("Buttons:")
  for k,v in btn_counts.most_common():
    print(f"  {k}: {v}")

  print("\nLast 20 rows:")
  for r in rows[-20:]:
    print(f"{r.get('ts_wall')} {r.get('tag')} {r.get('msg')}")

if __name__ == "__main__":
  main()
