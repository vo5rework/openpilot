#!/usr/bin/env python3
"""
Tesla 0x659 / IC-HUD diagnostic watcher for XNOR.

What it watches
- can 0x45 (real STW_ACTN_RQ stalk)
- sendcan 0x659 (internal carrier consumed by Tesla legacy safety)
- sendcan / can HUD+IC frames tied to blue-D / AP status presentation:
    0x399 DAS_status
    0x389 DAS_status2
    0x3E9 DAS_bodyControls
    0x329 / 0x349 / 0x369 warning matrices
- carState, pandaStates, selfdriveState, controlsState, logMessage (when available)

Why this exists
- If 0x659 is present and healthy, but HUD/IC frames are never emitted while OP is enabled,
  the blue D / cluster integration path is likely missing upstream of safety.
- If 0x659 is missing, the issue is earlier: userspace carrier generation / sendcan path.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from cereal import messaging


RUN = True

ADDR_STALK = 0x045
ADDR_659 = 0x659
ADDR_DAS_STATUS = 0x399
ADDR_DAS_STATUS2 = 0x389
ADDR_BODY = 0x3E9
ADDR_WARN0 = 0x329
ADDR_WARN3 = 0x349
ADDR_WARN1 = 0x369

WATCH_ADDRS = {
  ADDR_659: "0x659",
  ADDR_DAS_STATUS: "0x399",
  ADDR_DAS_STATUS2: "0x389",
  ADDR_BODY: "0x3E9",
  ADDR_WARN0: "0x329",
  ADDR_WARN3: "0x349",
  ADDR_WARN1: "0x369",
}

AP_STATE_NAMES = {
  0: "disabled",
  1: "unavailable",
  2: "available",
  3: "active_nominal",
  4: "active_restricted",
  5: "active_nav",
  8: "aborting",
  9: "aborted",
  14: "fault",
  15: "sna",
}


def _sig_handler(sig, frame) -> None:
  del sig, frame
  global RUN
  RUN = False


def safe_get(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def nested_get(obj: Any, path: str, default: Any = None) -> Any:
  cur = obj
  for part in path.split("."):
    try:
      cur = getattr(cur, part)
    except Exception:
      return default
  return cur


def safe_bool(value: Any, default: bool = False) -> bool:
  try:
    return bool(value)
  except Exception:
    return bool(default)


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    x = float(value)
    return x if math.isfinite(x) else float(default)
  except Exception:
    return float(default)


def safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return int(default)


def gear_name(value: Any) -> str:
  if value is None:
    return ""
  try:
    name = getattr(value, "name", None)
    if name:
      return str(name)
  except Exception:
    pass
  text = str(value)
  if "." in text:
    text = text.split(".")[-1]
  return text


def lever_position(dat: bytes) -> int:
  return (dat[0] & 0x3F) if dat else -1


def parse_659(dat: bytes) -> dict[str, int]:
  b5 = dat[5] if len(dat) > 5 else 0
  return {
    "b5": int(b5),
    "ap_disabled": 1 if (b5 & 0x80) else 0,
    "pedal_enabled": 1 if (b5 & 0x20) else 0,
    "main_edge": 1 if (b5 & 0x02) else 0,
    "cancel_edge": 1 if (b5 & 0x01) else 0,
  }


def parse_das_status(dat: bytes) -> dict[str, int]:
  b0 = dat[0] if len(dat) > 0 else 0
  b1 = dat[1] if len(dat) > 1 else 0
  b5 = dat[5] if len(dat) > 5 else 0
  return {
    "ap_state": int((b0 >> 4) & 0x0F),
    "collision_warning": int(b0 & 0x0F),
    "fused_speed_limit_raw": int(b1 & 0x1F),
    "counter": int((b5 >> 4) & 0x0F),
  }


def parse_das_status2(dat: bytes) -> dict[str, int]:
  b0 = dat[0] if len(dat) > 0 else 0
  b6 = dat[6] if len(dat) > 6 else 0
  return {
    "acc_report": int((b0 >> 2) & 0x1F),
    "long_collision_warning": int(b6 & 0x0F),
  }


def parse_body(dat: bytes) -> dict[str, int]:
  b0 = dat[0] if len(dat) > 0 else 0
  b1 = dat[1] if len(dat) > 1 else 0
  return {
    "hazard": int((b0 >> 2) & 0x03),
    "turn": int(b1 & 0x03),
  }


@dataclass
class WatchState:
  last_stalk_bus: int = -1
  last_stalk_lever: int = 0
  last_659_bus: int = -1
  last_659_b5: int = 0
  last_659_ap_disabled: int = 0
  last_659_pedal_enabled: int = 0
  last_659_main_edge: int = 0
  last_659_cancel_edge: int = 0

  sendcan_seen: dict[int, int] = field(default_factory=lambda: {addr: 0 for addr in WATCH_ADDRS})
  sendcan_buses: dict[int, set[int]] = field(default_factory=lambda: {addr: set() for addr in WATCH_ADDRS})

  can_seen: dict[int, int] = field(default_factory=lambda: {addr: 0 for addr in WATCH_ADDRS if addr != ADDR_659})
  can_buses: dict[int, set[int]] = field(default_factory=lambda: {addr: set() for addr in WATCH_ADDRS if addr != ADDR_659})

  last_ap_state_sendcan: int = -1
  last_ap_state_can: int = -1
  last_body_turn_sendcan: int = 0
  last_body_turn_can: int = 0
  last_warn_addr_sendcan: int = 0
  last_warn_addr_can: int = 0

  last_log_match: str = ""
  last_log_ts_wall: float = 0.0

  def clear_window(self) -> None:
    for addr in self.sendcan_seen:
      self.sendcan_seen[addr] = 0
      self.sendcan_buses[addr].clear()
    for addr in self.can_seen:
      self.can_seen[addr] = 0
      self.can_buses[addr].clear()
    self.last_warn_addr_sendcan = 0
    self.last_warn_addr_can = 0


def choose_services() -> list[str]:
  services = ["can", "sendcan", "carState", "pandaStates"]
  for name in ("selfdriveState", "controlsState", "logMessage"):
    if name in messaging.SERVICE_LIST:
      services.append(name)
  return services


def infer_note(row: dict[str, object]) -> str:
  sd_enabled = safe_bool(row.get("selfdrive_enabled", False))
  carrier_seen = safe_int(row.get("sendcan_659_count", 0)) > 0
  hud_send_count = (
    safe_int(row.get("sendcan_399_count", 0)) +
    safe_int(row.get("sendcan_389_count", 0)) +
    safe_int(row.get("sendcan_3e9_count", 0))
  )
  can_hud_count = (
    safe_int(row.get("can_399_count", 0)) +
    safe_int(row.get("can_389_count", 0))
  )
  controls_allowed_any = safe_int(row.get("panda_controls_allowed_count", 0)) > 0
  gear = str(row.get("gear", ""))
  ap_state_sendcan = safe_int(row.get("sendcan_399_ap_state", -1))
  ap_state_can = safe_int(row.get("can_399_ap_state", -1))

  if sd_enabled and not carrier_seen:
    return "OP enabled but no sendcan 0x659 seen -> carrier generation/path issue"
  if sd_enabled and carrier_seen and hud_send_count == 0:
    return "0x659 present but no sendcan HUD/IC frames -> likely missing Tesla HUD integration"
  if sd_enabled and carrier_seen and hud_send_count > 0 and not controls_allowed_any:
    return "HUD path active but panda controlsAllowed stayed false"
  if sd_enabled and carrier_seen and gear != "drive":
    return "OP enabled outside drive gear"
  if can_hud_count > 0 and ap_state_can in (3, 4, 5, 8):
    return "stock AP-side DAS_status active on CAN"
  if ap_state_sendcan in (2, 3, 4, 5) and hud_send_count > 0:
    return "HUD/IC path emitting OP DAS_status"
  return "no obvious blocker in this window"


def main() -> int:
  parser = argparse.ArgumentParser(description="Tesla 0x659 + blue-D / IC-HUD watcher for XNOR")
  parser.add_argument("--seconds", type=float, default=0.0, help="Run duration; 0 means until Ctrl+C")
  parser.add_argument("--rate", type=float, default=10.0, help="CSV/sample rate in Hz")
  parser.add_argument("--summary-hz", type=float, default=1.0, help="Console summary rate in Hz")
  parser.add_argument("--out", default="", help="CSV output path")
  parser.add_argument("--print-events", action="store_true", help="Print raw watch events as they happen")
  args = parser.parse_args()

  signal.signal(signal.SIGINT, _sig_handler)
  signal.signal(signal.SIGTERM, _sig_handler)

  out = args.out.strip()
  if not out:
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = f"/data/media/0/realdata/tesla_0x659_watch_{ts}.csv"

  out_path = Path(out)
  out_path.parent.mkdir(parents=True, exist_ok=True)

  services = choose_services()
  sm = messaging.SubMaster(services, ignore_avg_freq=True)

  fieldnames = [
    "ts_wall",
    "frame",
    "selfdrive_enabled",
    "selfdrive_active",
    "selfdrive_engageable",
    "selfdrive_state",
    "controls_enabled",
    "gear",
    "cruise_enabled",
    "cruise_available",
    "cruise_speed_ms",
    "v_ego_ms",
    "gas_pressed",
    "brake_pressed",
    "last_stalk_bus",
    "last_stalk_lever",
    "sendcan_659_count",
    "sendcan_659_buses",
    "sendcan_659_last_bus",
    "sendcan_659_b5",
    "sendcan_659_ap_disabled",
    "sendcan_659_pedal_enabled",
    "sendcan_659_main_edge",
    "sendcan_659_cancel_edge",
    "sendcan_399_count",
    "sendcan_399_buses",
    "sendcan_399_ap_state",
    "sendcan_399_ap_state_name",
    "sendcan_389_count",
    "sendcan_3e9_count",
    "sendcan_3e9_turn",
    "sendcan_warn_count",
    "can_399_count",
    "can_399_buses",
    "can_399_ap_state",
    "can_399_ap_state_name",
    "can_389_count",
    "can_3e9_count",
    "can_3e9_turn",
    "can_warn_count",
    "panda_count",
    "panda_controls_allowed_count",
    "panda_faults",
    "panda_safety_params",
    "last_log_match",
    "last_log_age_s",
    "note",
  ]

  state = WatchState()
  started = time.monotonic()
  next_summary = started
  next_row = started
  frame = 0

  print(f"Watching services={services}")
  print(f"Writing CSV to {out_path}")
  print("Focus: 0x659 carrier + DAS_status/DAS_status2/body-controls presence while enabling OP.\n")

  with out_path.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()

    while RUN:
      sm.update(50)

      if sm.updated.get("can", False):
        for m in sm["can"]:
          addr = safe_int(safe_get(m, "address", 0))
          src = safe_int(safe_get(m, "src", -1))
          dat = bytes(safe_get(m, "dat", b""))
          if addr == ADDR_STALK:
            state.last_stalk_bus = src
            state.last_stalk_lever = lever_position(dat)
            if args.print_events and state.last_stalk_lever != 0:
              print(f"[can 0x45] bus={src} lever={state.last_stalk_lever} dat={dat.hex()}")

          if addr in state.can_seen:
            state.can_seen[addr] += 1
            state.can_buses[addr].add(src)
            if addr == ADDR_DAS_STATUS:
              parsed = parse_das_status(dat)
              state.last_ap_state_can = parsed["ap_state"]
              if args.print_events:
                print(f"[can 0x399] bus={src} ap_state={parsed['ap_state']}({AP_STATE_NAMES.get(parsed['ap_state'], 'unk')}) dat={dat.hex()}")
            elif addr == ADDR_BODY:
              parsed = parse_body(dat)
              state.last_body_turn_can = parsed["turn"]
              if args.print_events:
                print(f"[can 0x3E9] bus={src} turn={parsed['turn']} hazard={parsed['hazard']} dat={dat.hex()}")
            elif addr in (ADDR_WARN0, ADDR_WARN1, ADDR_WARN3):
              state.last_warn_addr_can = addr
              if args.print_events:
                print(f"[can {WATCH_ADDRS[addr]}] bus={src} dat={dat.hex()}")

      if sm.updated.get("sendcan", False):
        for m in sm["sendcan"]:
          addr = safe_int(safe_get(m, "address", 0))
          src = safe_int(safe_get(m, "src", -1))
          dat = bytes(safe_get(m, "dat", b""))
          if addr in state.sendcan_seen:
            state.sendcan_seen[addr] += 1
            state.sendcan_buses[addr].add(src)

          if addr == ADDR_659:
            parsed = parse_659(dat)
            state.last_659_bus = src
            state.last_659_b5 = parsed["b5"]
            state.last_659_ap_disabled = parsed["ap_disabled"]
            state.last_659_pedal_enabled = parsed["pedal_enabled"]
            state.last_659_main_edge = parsed["main_edge"]
            state.last_659_cancel_edge = parsed["cancel_edge"]
            if args.print_events:
              print(
                f"[sendcan 0x659] bus={src} b5=0x{parsed['b5']:02x} "
                f"ap_dis={parsed['ap_disabled']} pedal={parsed['pedal_enabled']} "
                f"main={parsed['main_edge']} cancel={parsed['cancel_edge']}"
              )
          elif addr == ADDR_DAS_STATUS:
            parsed = parse_das_status(dat)
            state.last_ap_state_sendcan = parsed["ap_state"]
            if args.print_events:
              print(f"[sendcan 0x399] bus={src} ap_state={parsed['ap_state']}({AP_STATE_NAMES.get(parsed['ap_state'], 'unk')}) dat={dat.hex()}")
          elif addr == ADDR_BODY:
            parsed = parse_body(dat)
            state.last_body_turn_sendcan = parsed["turn"]
            if args.print_events:
              print(f"[sendcan 0x3E9] bus={src} turn={parsed['turn']} hazard={parsed['hazard']} dat={dat.hex()}")
          elif addr in (ADDR_WARN0, ADDR_WARN1, ADDR_WARN3):
            state.last_warn_addr_sendcan = addr
            if args.print_events:
              print(f"[sendcan {WATCH_ADDRS[addr]}] bus={src} dat={dat.hex()}")

      if "logMessage" in services and sm.updated.get("logMessage", False):
        lm = sm["logMessage"]
        msg = str(safe_get(lm, "msg", "") or "")
        if any(tag in msg for tag in ("XNOR_CC_DIAG", "tesla", "0x659", "DAS_status", "HUD")):
          state.last_log_match = msg.strip().replace("\n", " ")[:240]
          state.last_log_ts_wall = time.time()

      now = time.monotonic()
      if now >= next_row:
        cs = sm["carState"]
        cruise = safe_get(cs, "cruiseState", None)
        ss = sm["selfdriveState"] if "selfdriveState" in services else None
        controls = sm["controlsState"] if "controlsState" in services else None
        pandas = list(sm["pandaStates"]) if sm.valid.get("pandaStates", False) else []

        panda_faults = "|".join(str(safe_get(p, "faultStatus", "")) for p in pandas)
        panda_safety_params = "|".join(str(safe_get(p, "safetyParam", "")) for p in pandas)
        panda_controls_allowed_count = sum(1 for p in pandas if safe_bool(safe_get(p, "controlsAllowed", False)))

        gear = gear_name(safe_get(cs, "gearShifter", ""))

        row = {
          "ts_wall": f"{time.time():.3f}",
          "frame": frame,
          "selfdrive_enabled": int(safe_bool(safe_get(ss, "enabled", False))),
          "selfdrive_active": int(safe_bool(safe_get(ss, "active", False))),
          "selfdrive_engageable": int(safe_bool(safe_get(ss, "engageable", False))),
          "selfdrive_state": safe_int(safe_get(ss, "state", -1), -1),
          "controls_enabled": int(safe_bool(safe_get(controls, "enabled", False))),
          "gear": gear,
          "cruise_enabled": int(safe_bool(safe_get(cruise, "enabled", False))),
          "cruise_available": int(safe_bool(safe_get(cruise, "available", False))),
          "cruise_speed_ms": safe_float(safe_get(cruise, "speed", 0.0)),
          "v_ego_ms": safe_float(safe_get(cs, "vEgo", 0.0)),
          "gas_pressed": int(safe_bool(safe_get(cs, "gasPressed", False))),
          "brake_pressed": int(safe_bool(safe_get(cs, "brakePressed", False))),
          "last_stalk_bus": state.last_stalk_bus,
          "last_stalk_lever": state.last_stalk_lever,
          "sendcan_659_count": state.sendcan_seen[ADDR_659],
          "sendcan_659_buses": ",".join(str(x) for x in sorted(state.sendcan_buses[ADDR_659])),
          "sendcan_659_last_bus": state.last_659_bus,
          "sendcan_659_b5": f"0x{state.last_659_b5:02x}",
          "sendcan_659_ap_disabled": state.last_659_ap_disabled,
          "sendcan_659_pedal_enabled": state.last_659_pedal_enabled,
          "sendcan_659_main_edge": state.last_659_main_edge,
          "sendcan_659_cancel_edge": state.last_659_cancel_edge,
          "sendcan_399_count": state.sendcan_seen[ADDR_DAS_STATUS],
          "sendcan_399_buses": ",".join(str(x) for x in sorted(state.sendcan_buses[ADDR_DAS_STATUS])),
          "sendcan_399_ap_state": state.last_ap_state_sendcan,
          "sendcan_399_ap_state_name": AP_STATE_NAMES.get(state.last_ap_state_sendcan, "unknown"),
          "sendcan_389_count": state.sendcan_seen[ADDR_DAS_STATUS2],
          "sendcan_3e9_count": state.sendcan_seen[ADDR_BODY],
          "sendcan_3e9_turn": state.last_body_turn_sendcan,
          "sendcan_warn_count": (
            state.sendcan_seen[ADDR_WARN0] +
            state.sendcan_seen[ADDR_WARN1] +
            state.sendcan_seen[ADDR_WARN3]
          ),
          "can_399_count": state.can_seen[ADDR_DAS_STATUS],
          "can_399_buses": ",".join(str(x) for x in sorted(state.can_buses[ADDR_DAS_STATUS])),
          "can_399_ap_state": state.last_ap_state_can,
          "can_399_ap_state_name": AP_STATE_NAMES.get(state.last_ap_state_can, "unknown"),
          "can_389_count": state.can_seen[ADDR_DAS_STATUS2],
          "can_3e9_count": state.can_seen[ADDR_BODY],
          "can_3e9_turn": state.last_body_turn_can,
          "can_warn_count": (
            state.can_seen[ADDR_WARN0] +
            state.can_seen[ADDR_WARN1] +
            state.can_seen[ADDR_WARN3]
          ),
          "panda_count": len(pandas),
          "panda_controls_allowed_count": panda_controls_allowed_count,
          "panda_faults": panda_faults,
          "panda_safety_params": panda_safety_params,
          "last_log_match": state.last_log_match,
          "last_log_age_s": f"{max(0.0, time.time() - state.last_log_ts_wall):.2f}" if state.last_log_ts_wall > 0.0 else "",
        }
        row["note"] = infer_note(row)

        writer.writerow(row)
        f.flush()
        frame += 1
        next_row = now + (1.0 / max(args.rate, 1.0))

      if now >= next_summary:
        cs = sm["carState"]
        ss = sm["selfdriveState"] if "selfdriveState" in services else None
        pandas = list(sm["pandaStates"]) if sm.valid.get("pandaStates", False) else []
        panda_controls_allowed_count = sum(1 for p in pandas if safe_bool(safe_get(p, "controlsAllowed", False)))
        gear = gear_name(safe_get(cs, "gearShifter", ""))
        sd_enabled = int(safe_bool(safe_get(ss, "enabled", False)))
        print(
          f"[summary] sd_en={sd_enabled} gear={gear} "
          f"659={state.sendcan_seen[ADDR_659]} buses={sorted(state.sendcan_buses[ADDR_659])} "
          f"b5=0x{state.last_659_b5:02x} "
          f"399_send={state.sendcan_seen[ADDR_DAS_STATUS]} "
          f"389_send={state.sendcan_seen[ADDR_DAS_STATUS2]} "
          f"3E9_send={state.sendcan_seen[ADDR_BODY]} "
          f"399_can={state.can_seen[ADDR_DAS_STATUS]} ap_can={AP_STATE_NAMES.get(state.last_ap_state_can, 'unknown')} "
          f"note={infer_note({'selfdrive_enabled': sd_enabled, 'sendcan_659_count': state.sendcan_seen[ADDR_659], 'sendcan_399_count': state.sendcan_seen[ADDR_DAS_STATUS], 'sendcan_389_count': state.sendcan_seen[ADDR_DAS_STATUS2], 'sendcan_3e9_count': state.sendcan_seen[ADDR_BODY], 'can_399_count': state.can_seen[ADDR_DAS_STATUS], 'can_389_count': state.can_seen[ADDR_DAS_STATUS2], 'panda_controls_allowed_count': panda_controls_allowed_count, 'gear': gear, 'sendcan_399_ap_state': state.last_ap_state_sendcan, 'can_399_ap_state': state.last_ap_state_can})}"
        )
        state.clear_window()
        next_summary = now + (1.0 / max(args.summary_hz, 0.1))

      if args.seconds > 0.0 and (now - started) >= args.seconds:
        break

    print(f"\nDone. CSV: {out_path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
