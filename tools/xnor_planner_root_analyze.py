#!/usr/bin/env python3
import argparse
import csv
from collections import Counter, defaultdict


def main() -> int:
  parser = argparse.ArgumentParser(description="Analyze second-stage XNOR planner block watch CSV.")
  parser.add_argument("csv_path")
  args = parser.parse_args()

  rows = []
  with open(args.csv_path, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
      rows.append(row)

  counts = Counter()
  spans = defaultdict(float)

  prev_ts = None
  prev_blocker = None
  current_span = 0.0

  for row in rows:
    blocker = row.get("likely_blocker", "unknown") or "unknown"
    ts = float(row.get("ts_wall", "0") or 0.0)
    counts[blocker] += 1

    if prev_ts is None:
      prev_ts = ts
      prev_blocker = blocker
      continue

    dt = max(0.0, ts - prev_ts)
    if blocker == prev_blocker:
      current_span += dt
    else:
      spans[prev_blocker] = max(spans[prev_blocker], current_span)
      current_span = 0.0
      prev_blocker = blocker
    prev_ts = ts

  print("Counts:")
  for key, value in counts.most_common():
    print(f"  {key}: {value}")

  print("Longest continuous spans (s):")
  for key, value in sorted(spans.items(), key=lambda kv: kv[1], reverse=True):
    print(f"  {key}: {value:.1f}")

  print("\nTop explanations:")
  expl_counts = Counter((row.get("likely_blocker", "unknown"), row.get("explanation", "")) for row in rows)
  for (blocker, expl), value in expl_counts.most_common(10):
    print(f"  {blocker}: {value} -> {expl}")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
