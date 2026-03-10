
#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict

def main() -> None:
  parser = argparse.ArgumentParser(description="Summarize xnor_speedup_block_watch CSV")
  parser.add_argument("csv_path")
  args = parser.parse_args()

  counts = Counter()
  longest = defaultdict(float)
  last_ts = None
  last_blk = None
  start_ts = None

  with open(args.csv_path, newline="") as f:
    rows = list(csv.DictReader(f))

  for row in rows:
    blk = row["likely_blocker"]
    counts[blk] += 1
    ts = float(row["ts"])

    if last_blk is None:
      last_blk = blk
      start_ts = ts
    elif blk != last_blk:
      longest[last_blk] = max(longest[last_blk], ts - start_ts)
      last_blk = blk
      start_ts = ts

    last_ts = ts

  if last_blk is not None and start_ts is not None and last_ts is not None:
    longest[last_blk] = max(longest[last_blk], last_ts - start_ts)

  print("Counts:")
  for blk, count in counts.most_common():
    print(f"  {blk}: {count}")

  print("\nLongest continuous spans (s):")
  for blk, dur in sorted(longest.items(), key=lambda kv: kv[1], reverse=True):
    print(f"  {blk}: {dur:.1f}")

if __name__ == "__main__":
  main()
