#!/usr/bin/env python3
"""
Tesla HW2 capture tool (Unity-to-XNOR port aid).

Writes JSONL lines with:
- monotonic timestamp (seconds since start)
- src (bus)
- address
- message name (if DBC available)
- raw hex payload
- selected decoded fields for: DI_state, STW_ACTN_RQ, UI_gpsVehicleSpeed, UI_driverAssistMapData,
  UI_driverAssistRoadSign, DAS_status2

This script is deliberately defensive across forks:
- Params.get API differences
- capnp from_bytes context-manager differences

Usage:
  python3 tools/tesla_capture.py --duration 60 --out /data/openpilot/tesla_cap.jsonl
  python3 tools/tesla_capture.py --duration 10 --no-decode
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Any, Optional

import cereal.messaging as messaging
from cereal import car

try:
  from common.params import Params
except Exception:
  from openpilot.common.params import Params  # type: ignore

from opendbc.can.dbc import DBC as DbcFile
from opendbc.can.parser import get_raw_value

from opendbc.car.tesla.values import DBC as TESLA_DBC_MAP
from opendbc.car import Bus


def _params_get_bytes(key: str) -> Optional[bytes]:
  p = Params()
  # Try multiple signatures
  for kwargs in (
    {"block": False, "encoding": None},
    {"block": False},
    {"block": True, "encoding": None},
    {"block": True},
  ):
    try:
      raw = p.get(key, **kwargs)  # type: ignore[arg-type]
    except TypeError:
      continue
    if raw is None:
      continue
    if isinstance(raw, bytes):
      return raw
    if isinstance(raw, str):
      return raw.encode("utf-8", errors="ignore")
    try:
      return bytes(raw)
    except Exception:
      return None

  # Direct file fallback
  for path in (f"/data/params/d/{key}", f"/data/params/{key}"):
    try:
      with open(path, "rb") as f:
        return f.read()
    except Exception:
      pass
  return None


def _carparams_from_bytes(raw: bytes) -> Any:
  obj = car.CarParams.from_bytes(raw)
  if hasattr(obj, "__enter__") and hasattr(obj, "__exit__"):
    with obj as cp:
      return cp
  return obj


def _load_fingerprint() -> str:
  raw = _params_get_bytes("CarParams")
  if raw is None:
    raise RuntimeError("CarParams not available. Run after openpilot has started.")
  cp = _carparams_from_bytes(raw)
  return str(cp.carFingerprint)


def _safe_decode(dbc: DbcFile, addr: int, dat: bytes) -> tuple[Optional[str], dict[str, Any]]:
  msg = dbc.addr_to_msg.get(int(addr))
  if msg is None:
    return None, {}
  out: dict[str, Any] = {}
  for sig_name, sig in msg.sigs.items():
    raw = get_raw_value(dat, sig)
    if sig.is_signed and (raw & (1 << (sig.size - 1))):
      raw = raw - (1 << sig.size)
    out[sig_name] = raw * sig.factor + sig.offset
  return msg.name, out


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--duration", type=float, default=30.0)
  ap.add_argument("--out", type=str, default="/data/openpilot/tesla_capture.jsonl")
  ap.add_argument("--no-decode", action="store_true", help="raw-only capture (no DBC decode)")
  ap.add_argument("--fingerprint", type=str, default=None, help="override car fingerprint")
  ap.add_argument("--dbc", type=str, default=None, help="override dbc name directly")
  args = ap.parse_args()

  fp = args.fingerprint or _load_fingerprint()
  dbc_name = args.dbc or str(TESLA_DBC_MAP[fp][Bus.party])
  dbc = None if args.no_decode else DbcFile(dbc_name)

  want_names = {
    "DI_state",
    "STW_ACTN_RQ",
    "UI_gpsVehicleSpeed",
    "UI_driverAssistMapData",
    "UI_driverAssistRoadSign",
    "DAS_status2",
  }

  sock = messaging.sub_sock("can", conflate=False, timeout=1000)
  t0 = time.monotonic()

  os.makedirs(os.path.dirname(args.out), exist_ok=True)
  with open(args.out, "w", encoding="utf-8") as f:
    while True:
      if args.duration and (time.monotonic() - t0) > args.duration:
        break
      msg = messaging.recv_sock(sock)
      if msg is None:
        continue

      now = time.monotonic() - t0
      for cp in msg.can:
        addr = int(cp.address)
        src = int(cp.src)
        dat = bytes(cp.dat)
        raw_hex = dat.hex()

        name = None
        decoded: dict[str, Any] = {}
        if dbc is not None:
          name, decoded = _safe_decode(dbc, addr, dat)
          if name is None or name not in want_names:
            continue

        # Trim to keep files smaller
        if decoded and name == "DI_state":
          keep = ("DI_cruiseState", "DI_cruiseSet", "DI_speedUnits", "DI_digitalSpeed")
          decoded = {k: decoded.get(k) for k in keep if k in decoded}
        elif decoded and name == "STW_ACTN_RQ":
          keep = ("SpdCtrlLvr_Stat", "TurnIndLvr_Stat", "MC_STW_ACTN_RQ", "CRC_STW_ACTN_RQ")
          decoded = {k: decoded.get(k) for k in keep if k in decoded}
        elif decoded and name == "UI_gpsVehicleSpeed":
          keep = ("mppSpeedLimit", "units", "UI_mppSpeedLimit", "UI_mapSpeedLimitUnits", "gpsVehicleSpeed")
          decoded = {k: decoded.get(k) for k in keep if k in decoded}

        rec = {
          "t": round(now, 6),
          "src": src,
          "addr": addr,
          "name": name,
          "raw": raw_hex,
          "dec": decoded,
        }
        f.write(json.dumps(rec, separators=(",", ":")) + "\n")

  print(f"wrote: {args.out}")


if __name__ == "__main__":
  main()
