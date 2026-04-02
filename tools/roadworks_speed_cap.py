#!/usr/bin/env python3
# /data/openpilot/tools/roadworks_speed_cap.py
from __future__ import annotations

import argparse
import sys

from openpilot.common.params import Params

CAP_PARAM = "XNORRoadworksSpeedCapKph"
PRESET_PARAM = "XNORRoadworksSpeedCapPresetKph"

def mph_to_kph(v: float) -> float:
  return float(v) * 1.609344

def fmt_kph(kph: float | None) -> str:
  if kph is None:
    return "OFF"
  return f"{float(kph):.3f} kph"

def read_float(params: Params, key: str) -> float | None:
  try:
    raw = params.get(key, encoding="utf-8")
  except Exception:
    return None
  if not raw:
    return None
  try:
    value = float(str(raw).strip())
  except Exception:
    return None
  return value if value > 0.0 else None

def write_float(params: Params, key: str, value_kph: float) -> None:
  params.put(key, f"{float(value_kph):.3f}")

def main(argv: list[str]) -> int:
  parser = argparse.ArgumentParser(description="Manage XNOR temporary roadworks speed cap")
  sub = parser.add_subparsers(dest="cmd", required=True)

  p_set = sub.add_parser("set", help="Enable active temporary cap")
  p_set.add_argument("value", type=float)
  p_set.add_argument("--mph", action="store_true")
  p_set.add_argument("--kph", action="store_true")

  p_preset = sub.add_parser("preset", help="Set triple-pull preset cap")
  p_preset.add_argument("value", type=float)
  p_preset.add_argument("--mph", action="store_true")
  p_preset.add_argument("--kph", action="store_true")

  sub.add_parser("clear", help="Disable active temporary cap")
  sub.add_parser("status", help="Show active cap + preset")

  args = parser.parse_args(argv)
  params = Params()

  if args.cmd == "set":
    value_kph = mph_to_kph(args.value) if args.mph else float(args.value)
    write_float(params, CAP_PARAM, value_kph)
    print(f"active cap set: {fmt_kph(value_kph)}")
    return 0

  if args.cmd == "preset":
    value_kph = mph_to_kph(args.value) if args.mph else float(args.value)
    write_float(params, PRESET_PARAM, value_kph)
    print(f"preset cap set: {fmt_kph(value_kph)}")
    return 0

  if args.cmd == "clear":
    params.remove(CAP_PARAM)
    print("active cap cleared")
    return 0

  active = read_float(params, CAP_PARAM)
  preset = read_float(params, PRESET_PARAM)
  print(f"active={fmt_kph(active)} preset={fmt_kph(preset)}")
  return 0

if __name__ == "__main__":
  raise SystemExit(main(sys.argv[1:]))
