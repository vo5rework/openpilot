# /data/openpilot/selfdrive/car/modules/LONG_module.py
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from cereal import messaging
from openpilot.common.swaglog import cloudlog
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.modules.ACC_module import ACCController, AccDecision


def _mono_ms() -> int:
  return time.monotonic_ns() // 1_000_000


@dataclass
class LongDecision:
  button: Optional[int]
  log: str = ""


class LongController:
  _LP_FRESH_NS = 1_500_000_000

  def __init__(self) -> None:
    self.acc = ACCController()
    self._sm = messaging.SubMaster(["longitudinalPlan"])

    self._lp_target_ms: Optional[float] = None
    self._lp_last_ns: int = 0

    self._last_info_log_ms: int = 0

  def _rate_log(self, msg: str) -> None:
    now = _mono_ms()
    if now - int(self._last_info_log_ms) < 1000:
      return
    self._last_info_log_ms = int(now)
    cloudlog.info(msg)

  @staticmethod
  def _extract_plan_speed_last(lp) -> Optional[float]:
    speeds = getattr(lp, "speeds", None)
    if not speeds:
      return None
    try:
      v_last = float(speeds[-1])
    except Exception:
      return None
    if not (math.isfinite(v_last) and v_last >= 0.0):
      return None
    return float(v_last)

  def _poll_plan(self) -> None:
    try:
      self._sm.update(0)
      lp = self._sm["longitudinalPlan"]
      v_last = self._extract_plan_speed_last(lp)
      lp_mono_ns = int(self._sm.logMonoTime.get("longitudinalPlan", 0) or 0)
      if v_last is not None and lp_mono_ns > 0:
        self._lp_target_ms = float(v_last)
        self._lp_last_ns = int(lp_mono_ns)
    except Exception:
      pass

  @staticmethod
  def _resolve_speed_limit_target_ms(CS, *, speed_units: str) -> Optional[float]:
    tinkla = getattr(CS, "_tinkla", None)
    use_speed_limit = bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False))
    if not use_speed_limit:
      return None
    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0
    return float(speed_limit_target_ms) if speed_limit_target_ms > 0.1 else None

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)
    now_ns = int(now) * 1_000_000

    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    adaptive_enabled = bool(getattr(CS, "enable_adaptive_cruise", False) or getattr(CS, "enableACC", False))
    if (not bool(enabled)) or (not adaptive_enabled):
      self._rate_log(
        f"[XNOR_LONG_DIAG] gate=not_enabled enabled={int(bool(enabled))} "
        f"adaptive={int(adaptive_enabled)} enableACC={int(bool(getattr(CS, 'enableACC', False)))} "
        f"enable_adaptive={int(bool(getattr(CS, 'enable_adaptive_cruise', False)))}"
      )
      return LongDecision(None, "gated: not enabled/adaptive")

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "").upper()
    if stock_state not in ("ENABLED", "OVERRIDE", "STANDSTILL", "STANDBY"):
      self._rate_log(f"[XNOR_LONG_DIAG] gate=stock_state stock_state={stock_state or 'UNKNOWN'}")
      return LongDecision(None, f"gated: stock_state={stock_state or 'UNKNOWN'}")

    cs_out = getattr(CS, "out", None)
    v_ego_ms = float(getattr(cs_out, "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")
    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    self._poll_plan()
    lp_fresh = (
      (self._lp_target_ms is not None)
      and (int(self._lp_last_ns) > 0)
      and ((now_ns - int(self._lp_last_ns)) < int(self._LP_FRESH_NS))
    )
    desired_ms = float(self._lp_target_ms) if lp_fresh and self._lp_target_ms is not None else float(current_set_ms)
    speed_limit_target_ms = self._resolve_speed_limit_target_ms(CS, speed_units=speed_units)

    if not lp_fresh:
      self._rate_log(
        f"[XNOR_LONG_DIAG] gate=lp_not_fresh vEgo={float(v_ego_ms):.2f} "
        f"set_ms={float(current_set_ms):.2f} desired_ms={float(desired_ms):.2f} "
        f"lp_target_ms={float(self._lp_target_ms) if self._lp_target_ms is not None else -1.0:.2f}"
      )

    decision: AccDecision = self.acc.update(
      now_ms=int(now),
      enabled=True,
      adaptive_enabled=adaptive_enabled,
      stock_cruise_state=stock_state,
      speed_units=speed_units,
      v_ego_ms=float(v_ego_ms),
      current_set_speed_ms=float(current_set_ms),
      desired_speed_ms=float(desired_ms),
      cruise_buttons=int(cruise_buttons),
      speed_limit_target_ms=speed_limit_target_ms,
      brake_pressed=bool(getattr(cs_out, "brakePressed", False)),
    )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      src = "lp_last" if lp_fresh else "hold"
      self._rate_log(
        f"[XNOR_LONG_DIAG] gate=no_button reason={decision.reason} src={src} "
        f"vEgo={float(v_ego_ms):.2f} set_ms={float(current_set_ms):.2f} desired_ms={float(desired_ms):.2f} "
        f"speed_limit_ms={float(speed_limit_target_ms) if speed_limit_target_ms is not None else -1.0:.2f} "
        f"stock_state={stock_state}"
      )
      return LongDecision(None, f"{decision.reason} src={src}")

    msg = (
      f"[XNOR_CRUISE_SYNC] src={'lp_last' if lp_fresh else 'hold'} "
      f"tgt={decision.target_kph:.1f} cur={decision.current_kph:.1f} "
      f"est={decision.est_kph:.1f} btn={int(decision.button)} reason={decision.reason}"
    )
    self._rate_log(msg)
    return LongDecision(int(decision.button), msg)
