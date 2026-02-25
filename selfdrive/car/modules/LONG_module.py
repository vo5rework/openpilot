# /data/openpilot/selfdrive/car/modules/LONG_module.py
"""Unity-style speed-limit cruise syncing (XNOR).

This module decides which Tesla cruise stalk button to emulate (if any) to move the
stock cruise SET speed toward a desired target.

Key points (Unity outcome + fixes your regressions):
  - Only adjusts when DI_cruiseState == ENABLED.
  - Does NOT depend on longitudinalPlan (unstable in lateral-only setups and can cause random decels).
  - Smooths by ramping the *desired set-speed* toward the speed-limit target at a bounded rate.
  - On ENABLED rising edge, temporarily pulls set speed down toward current vEgo to avoid
    resuming at an old high set speed (Unity outcome you want).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from openpilot.common.swaglog import cloudlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.modules.ACC_module import ACCController, AccDecision


def _mono_ms() -> int:
  return time.monotonic_ns() // 1_000_000


@dataclass
class LongDecision:
  button: Optional[int]
  log: str = ""


class LongController:
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  # Smoothness knobs (bounded; expressed as mph/sec)
  RAMP_UP_MPH_PER_S = 2.0
  RAMP_DOWN_MPH_PER_S = 3.0

  def __init__(self) -> None:
    self.acc = ACCController()

    self._stock_enabled_prev = False
    self._last_eval_frame = -100000
    self._last_gate_log_ms = 0

    self._smooth_target_ms: Optional[float] = None
    self._engage_override_until_ms = 0

  def _gate_log(self, reason: str) -> None:
    now = _mono_ms()
    if now - int(self._last_gate_log_ms) < 1000:
      return
    self._last_gate_log_ms = int(now)
    cloudlog.info(f"[XNOR_CRUISE_GATE] {reason}")

  @staticmethod
  def _ramp(current: float, target: float, *, up_msps: float, down_msps: float, dt_s: float) -> float:
    if dt_s <= 0.0:
      return target
    if target > current:
      return min(target, current + up_msps * dt_s)
    return max(target, current - down_msps * dt_s)

  def update(self, CS, *, enabled: bool, frame: int, now_ms: Optional[int] = None) -> LongDecision:
    now = _mono_ms() if now_ms is None else int(now_ms)

    # Unity: extend pause while held.
    try:
      cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)
      self.acc.note_human_buttons(cruise_buttons, now_ms=now)
    except Exception:
      cruise_buttons = int(CruiseButtons.IDLE)

    # 5 Hz eval (Unity: frame % 20 on a 100 Hz loop).
    if (int(frame) % 20) != 0:
      return LongDecision(None, "gated: 5Hz(frame)")

    tinkla = getattr(CS, "_tinkla", None)
    if not bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False)):
      self._gate_log("adjust_acc_with_speed_limit false")
      return LongDecision(None, "gated: adjust_acc_with_speed_limit false")

    speed_units = str(getattr(CS, "speed_units", "MPH") or "MPH")

    # Speed limit (+offset) from CarState helper.
    try:
      speed_limit_target_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      speed_limit_target_ms = 0.0
    if speed_limit_target_ms <= 0.0:
      self._gate_log("no speed limit")
      return LongDecision(None, "gated: no speed limit")

    stock_state = str(getattr(CS, "stock_cruise_state", "") or "")
    stock_enabled = (stock_state == "ENABLED")
    if not stock_enabled:
      return LongDecision(None, f"gated: stock_state={stock_state}")

    v_ego_ms = float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    src = str(getattr(CS, "_cruise_set_src", "none") or "none")

    # ENABLED edge: mitigate resume-at-last-speed by pulling down toward current vEgo (capped by limit).
    if stock_enabled and (not bool(self._stock_enabled_prev)):
      self._engage_override_until_ms = int(now) + 1500
      # Allow immediate correction (do not wait for the 3s human pause)
      try:
        self.acc.human_action_time_ms = min(int(getattr(self.acc, "human_action_time_ms", 0)), int(now) - 3001)
      except Exception:
        pass
    self._stock_enabled_prev = bool(stock_enabled)

    desired_ms = float(speed_limit_target_ms)
    if int(now) < int(self._engage_override_until_ms):
      one_mph = 1.0 * CV.MPH_TO_MS
      if current_set_ms > (v_ego_ms + 0.6 * one_mph):
        desired_ms = float(min(speed_limit_target_ms, max(v_ego_ms, float(self.MIN_CRUISE_SPEED_MS))))
        src = f"{src}|engage_to_vEgo"
      else:
        self._engage_override_until_ms = 0

    # Initialize smoothing near current set to avoid a step.
    if self._smooth_target_ms is None:
      base = current_set_ms if current_set_ms > 0.1 else max(v_ego_ms, float(self.MIN_CRUISE_SPEED_MS))
      self._smooth_target_ms = float(min(speed_limit_target_ms, base))

    # Ramp smoothing target toward desired_ms.
    # (We keep the ramp expressed in MPH/s regardless of units; it's only about feel.)
    dt_s = 0.2  # frame%20 on 100Hz loop
    up_msps = float(self.RAMP_UP_MPH_PER_S) * CV.MPH_TO_MS
    down_msps = float(self.RAMP_DOWN_MPH_PER_S) * CV.MPH_TO_MS
    self._smooth_target_ms = float(self._ramp(float(self._smooth_target_ms), float(desired_ms), up_msps=up_msps, down_msps=down_msps, dt_s=dt_s))

    decision: AccDecision = self.acc.update(
      now_ms=now,
      enabled=bool(enabled),
      stock_cruise_enabled=True,
      speed_units=speed_units,
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_ms,
      desired_speed_ms=float(self._smooth_target_ms),
      cruise_buttons=cruise_buttons,
    )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, decision.reason)

    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    smooth_u = float(self._smooth_target_ms) * (CV.MS_TO_MPH if speed_units == "MPH" else CV.MS_TO_KPH)
    msg = (
      f"[XNOR_CRUISE_SYNC] uom={speed_units} "
      f"tgt={decision.target_kph*kph_to_u:.1f} "
      f"cur={decision.current_kph*kph_to_u:.1f} "
      f"est={decision.est_kph*kph_to_u:.1f} "
      f"btn={int(decision.button)} reason={decision.reason} src={src} smooth={smooth_u:.1f}"
    )
    cloudlog.info(msg)
    return LongDecision(int(decision.button), msg)
