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
  b5 = dat[5] if len(dat) > 5 else 0
  return {
    "b5": b5,
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
    "ts": time.time(),
  }


def main():
  parser = argparse.ArgumentParser(description="Raw XNOR speed-up path probe: decision -> sendcan -> readback")
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
    "stock_set_ms",
    "controls_vcruise_kph",
    "car_vcruise_kph",
    "lead_status",
    "lead_drel_m",
    "lead_vrel_ms",
    "lp_last_ms",
    "lp_has_lead",
    "gas_pressed",
    "brake_pressed",
    "panda0_controls_allowed",
    "xnor_log_recent",
    "xnor_btn",
    "xnor_reason",
    "xnor_src",
    "xnor_tgt",
    "xnor_cur",
    "xnor_est",
    "sendcan_659_seen",
    "sendcan_659_count_total",
    "sendcan_main_edge_total",
    "sendcan_cancel_edge_total",
    "sendcan_last_b5",
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
        try:
          msgs = sm["sendcan"]
        except Exception:
          msgs = []
        for m in msgs:
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

      xnor_recent = 0
      if last_xnor is not None and (time.time() - float(last_xnor.get("ts", 0.0))) < 0.75:
        xnor_recent = 1

      writer.writerow({
        "ts_wall": f"{time.time():.3f}",
        "v_ego_ms": f"{v_ego_ms:.3f}",
        "stock_set_ms": f"{stock_set_ms:.3f}",
        "controls_vcruise_kph": f"{controls_vcruise:.3f}",
        "car_vcruise_kph": f"{car_vcruise:.3f}",
        "lead_status": lead_status,
        "lead_drel_m": f"{lead_drel:.3f}",
        "lead_vrel_ms": f"{lead_vrel:.3f}",
        "lp_last_ms": f"{lp_last:.3f}",
        "lp_has_lead": lp_has_lead,
        "gas_pressed": gas_pressed,
        "brake_pressed": brake_pressed,
        "panda0_controls_allowed": panda0_controls_allowed,
        "xnor_log_recent": xnor_recent,
        "xnor_btn": int(last_xnor.get("btn", 0)) if last_xnor else 0,
        "xnor_reason": last_xnor.get("reason", "") if last_xnor else "",
        "xnor_src": last_xnor.get("src", "") if last_xnor else "",
        "xnor_tgt": f"{float(last_xnor.get('tgt', 0.0)):.3f}" if last_xnor else "0.000",
        "xnor_cur": f"{float(last_xnor.get('cur', 0.0)):.3f}" if last_xnor else "0.000",
        "xnor_est": f"{float(last_xnor.get('est', 0.0)):.3f}" if last_xnor else "0.000",
        "sendcan_659_seen": sendcan_seen_this_row,
        "sendcan_659_count_total": sendcan_659_count,
        "sendcan_main_edge_total": sendcan_main_edge_count,
        "sendcan_cancel_edge_total": sendcan_cancel_edge_count,
        "sendcan_last_b5": f"0x{sendcan_last_b5:02x}",
      })
      fcsv.flush()

      dt = time.time() - t0
      if dt < interval:
        time.sleep(interval - dt)


if __name__ == "__main__":
  main()
