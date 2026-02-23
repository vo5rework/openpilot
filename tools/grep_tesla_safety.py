#!/usr/bin/env python3
"""Locate Tesla legacy safetyParam interpretation in your local repo (no guessing).

Prints file/line/context for hits on:
- teslaLegacy / TESLA_LEGACY
- SAFETY_TESLA
- safetyParam / safety_param

Run:
  python3 /data/openpilot/tools/grep_tesla_safety.py
"""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Iterable

ROOTS = [
  "/data/openpilot/selfdrive/pandad",
  "/data/openpilot/openpilot/selfdrive/pandad",
  "/data/openpilot/selfdrive",
  "/data/openpilot/openpilot/selfdrive",
]

EXTS = {".c", ".cc", ".cpp", ".h", ".hpp", ".py", ".pyx", ".pxd", ".txt"}

PAT = re.compile(r"(teslaLegacy|TESLA_LEGACY|SAFETY_TESLA|safetyParam|safety_param|TESLA)", re.IGNORECASE)

def iter_files() -> Iterable[Path]:
  seen = set()
  for r in ROOTS:
    rp = Path(r)
    if not rp.exists():
      continue
    for p in rp.rglob("*"):
      if not p.is_file():
        continue
      if p.suffix.lower() not in EXTS:
        continue
      # skip large binaries
      try:
        if p.stat().st_size > 2_000_000:
          continue
      except Exception:
        continue
      sp = str(p)
      if sp in seen:
        continue
      seen.add(sp)
      yield p

def main() -> None:
  hits = 0
  for p in iter_files():
    try:
      txt = p.read_text(encoding="utf-8", errors="ignore").splitlines()
    except Exception:
      continue
    for i, line in enumerate(txt):
      if not PAT.search(line):
        continue
      hits += 1
      lo = max(0, i - 3)
      hi = min(len(txt), i + 4)
      print(f"\n=== {p} :{i+1} ===")
      for j in range(lo, hi):
        pref = ">>" if j == i else "  "
        print(f"{pref} {j+1:5d}: {txt[j]}")
  if hits == 0:
    print("No matches found under:", ", ".join([r for r in ROOTS if Path(r).exists()]))
    print("If your fork stores safety elsewhere, tell me the path and I'll adjust this script.")

if __name__ == "__main__":
  main()
