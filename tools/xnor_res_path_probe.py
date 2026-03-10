#!/usr/bin/env python3
import argparse
import csv
import re
import time
from pathlib import Path

import cereal.messaging as messaging

ADDR_659 = 0x659

BTN_RE = re.compile(r"btn=(\d+)")
REASON_RE = re.compile(r"reason=([^\s]+)")
SRC_RE = re.compile(r"src=([^\s]+)")
TGT_RE = re.compile(r"tgt=([0-9.]+)")
CUR_RE = re.compile(r"cur=([0-9.]+)")
EST_RE = re.compile(r"est=([0-9.]+)")


def safe_get(obj, name, default=None):
  try:
    return getattr(obj, name)
  except Exception:
    return default


def safe_get_path(obj, path, default=None):
  cur = obj
  for p in path.split("."):
    cur = safe_get(cur, p, default=None)
    if cur is None:
      return default
  return cur


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


def b(x):
  try:
    return int(bool(x))
  except Exception:
    return 0


def parse_659(dat: bytes):
  # Keep it simple: just record that 0x659 was sent, plus byte5 bits.
  b5 = dat[5] if len(dat) > 5 else 0
  return {
    "b5": b5,
    "ap_dis": int(bool(b5 & 0x80)),
    "ped_en": int(bool(b5 & 0x20)),
    "main_edge": int(bool(b5 & 0x02)),
    "cancel_edge": int(bool(b5 & 0x01)),
  }


def parse_xnor_log(msg: str):
  if "XNOR_CRUISE_SYNC" not in msg:
    return None
  m_btn = BTN_RE.search(msg)
  m_reason = REASON_RE.search(msg)
  m_src = SRC_RE.search(msg)
  m_tgt = TGT_RE.search(msg)
  m_cur = CUR_RE.search(msg)
  m_est = EST_RE.search(msg)
  return {
    "btn": int(m_btn.group(1)) if m_btn else 0,
    "reason": m_reason.group(1) if m_reason else "",
    "src": m_src.group(1) if m_src else "",
    "tgt": float(m_tgt.group(1)) if m_tgt else 0.0,
    "cur": float(m_cur.group(1)) if m_cur else 0.0,
    "est": float(m_est.group(1)) if m_est else 0.0,
    "msg": msg,
  }


def main():
  parser = argparse.ArgumentParser(description="Probe XNOR speed-up path: module decision -> sendcan -> readback")
  parser.add_argument("--out", default="/data/media/0/realdata/xnor_res_path_probe.csv")
  parser.add_argument("--rate", type=float, default=10.0)
  args = parser.parse_args()

  out_path = Path(args.out)
  out_path.parent.mkdir(parents=True, exist_ok=True)

  sm = messaging.SubMaster(
    ["carState", "controlsState", "radarState", "longitudinalPlan", "sendcan", "logMessage", "pandaStates"],
    ignore_avg_freq=True,
  )

  fieldnames = [
    "ts_wall",
    "v_ego_ms",
    "stock_cruise_enabled",
    "stock_set_ms",
    "controls_vcruise",
    "car_vcruise",
    "lead_status",
    "lead_drel",
    "lead_vrel",
    "lp_last",
    "lp_has_lead",
    "gas_pressed",
    "brake_pressed",
    "panda0_controls_allowed",
    "xnor_log_seen",
    "xnor_btn",
    "xnor_reason",
    "xnor_src",
    "xnor_tgt",
    "xnor_cur",
    "xnor_est",
    "sendcan_659_seen",
    "sendcan_659_count",
    "sendcan_main_edge_count",
    "sendcan_cancel_edge_count",
    "sendcan_last_b5",
    "notes",
  ]

  last_xnor = None
  sendcan_659_count = 0
  sendcan_main_edge_count = 0
  sendcan_cancel_edge_count = 0
  sendcan_last_b5 = 0

  interval = 1.0 / max(float(args.rate), 1.0)

  with out_path.open("w", newline="") as fcsv:
    writer = csv.DictWriter(fcsv, fieldnames=fieldnames)
    writer.writeheader()

    while True:
      t0 = time.time()
      sm.update(100)

      if sm.updated.get("logMessage", False):
        lm = sm["logMessage"]
        msg = safe_get(lm, "msg", "") or ""
        parsed = parse_xnor_log(msg)
        if parsed is not None:
          last_xnor = parsed

      sendcan_seen_this_row = 0
      if sm.updated.get("sendcan", False):
        for m in sm["sendcan"]:
          if i(safe_get(m, "address", 0), 0) == ADDR_659:
            sendcan_seen_this_row = 1
            sendcan_659_count += 1
            parsed_659 = parse_659(bytes(safe_get(m, "dat", b"")))
            sendcan_last_b5 = parsed_659["b5"]
            sendcan_main_edge_count += parsed_659["main_edge"]
            sendcan_cancel_edge_count += parsed_659["cancel_edge"]

      cs = sm["carState"]
      ctrl = sm["controlsState"]
      rs = sm["radarState"]
      lp = sm["longitudinalPlan"]
      lead = safe_get(rs, "leadOne", None)

      v_ego_ms = f(safe_get(cs, "vEgo", 0.0), 0.0)
      stock_cruise_enabled = b(safe_get_path(cs, "cruiseState.enabled", False))
      stock_set_ms = max(
        f(safe_get_path(cs, "cruiseState.speed", 0.0), 0.0),
        f(safe_get_path(cs, "cruiseState.speedCluster", 0.0), 0.0),
      )
      controls_vcruise = f(safe_get(ctrl, "vCruise", 0.0), 0.0)
      car_vcruise = f(safe_get(cs, "vCruise", 0.0), 0.0)

      lead_status = b(safe_get(lead, "status", False))
      lead_drel = f(safe_get(lead, "dRel", 0.0), 0.0)
      lead_vrel = f(safe_get(lead, "vRel", 0.0), 0.0)

      speeds = list(safe_get(lp, "speeds", []) or [])
      lp_last = f(speeds[-1], 0.0) if len(speeds) > 0 else 0.0
      lp_has_lead = b(safe_get(lp, "hasLead", False))

      gas_pressed = b(safe_get(cs, "gasPressed", False))
      brake_pressed = b(safe_get(cs, "brakePressed", False))

      panda0_controls_allowed = 0
      try:
        ps = sm["pandaStates"]
        if len(ps) > 0:
          panda0_controls_allowed = b(safe_get(ps[0], "controlsAllowed", False))
      except Exception:
        panda0_controls_allowed = 0

      notes = []
      demand_up = (max(controls_vcruise, car_vcruise) > 0.5 and max(controls_vcruise, car_vcruise) > stock_set_ms * 3.6 + 1.0) or (lp_last > stock_set_ms + 0.5)
      if demand_up:
        notes.append("speedup_demand")
      if last_xnor and int(last_xnor.get("btn", 0)) != 0:
        notes.append("xnor_decision")
      if sendcan_seen_this_row:
        notes.append("sendcan_659")

      writer.writerow({
        "ts_wall": f"{time.time():.3f}",
        "v_ego_ms": f"{v_ego_ms:.3f}",
        "stock_cruise_enabled": stock_cruise_enabled,
        "stock_set_ms": f"{stock_set_ms:.3f}",
        "controls_vcruise": f"{controls_vcruise:.3f}",
        "car_vcruise": f"{car_vcruise:.3f}",
        "lead_status": lead_status,
        "lead_drel": f"{lead_drel:.3f}",
        "lead_vrel": f"{lead_vrel:.3f}",
        "lp_last": f"{lp_last:.3f}",
        "lp_has_lead": lp_has_lead,
        "gas_pressed": gas_pressed,
        "brake_pressed": brake_pressed,
        "panda0_controls_allowed": panda0_controls_allowed,
        "xnor_log_seen": int(last_xnor is not None),
        "xnor_btn": int(last_xnor.get("btn", 0)) if last_xnor else 0,
        "xnor_reason": last_xnor.get("reason", "") if last_xnor else "",
        "xnor_src": last_xnor.get("src", "") if last_xnor else "",
        "xnor_tgt": f"{float(last_xnor.get('tgt', 0.0)):.3f}" if last_xnor else "0.000",
        "xnor_cur": f"{float(last_xnor.get('cur', 0.0)):.3f}" if last_xnor else "0.000",
        "xnor_est": f"{float(last_xnor.get('est', 0.0)):.3f}" if last_xnor else "0.000",
        "sendcan_659_seen": sendcan_seen_this_row,
        "sendcan_659_count": sendcan_659_count,
        "sendcan_main_edge_count": sendcan_main_edge_count,
        "sendcan_cancel_edge_count": sendcan_cancel_edge_count,
        "sendcan_last_b5": f"0x{sendcan_last_b5:02x}",
        "notes": "|".join(notes),
      })
      fcsv.flush()

      dt = time.time() - t0
      if dt < interval:
        time.sleep(interval - dt)


if __name__ == "__main__":
  main()
