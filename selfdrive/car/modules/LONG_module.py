"""
/data/openpilot/selfdrive/car/modules/LONG_module.py

Unity-style LONG module wrapper for XNOR.

Today: speed-limit-based stock cruise set speed nudging via CruiseButtons output.
Future: radar / follow distance / autoresume hooks.

This module does NOT send CAN. It returns a CruiseButtons value (or None).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from openpilot.common.swaglog import cloudlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.modules.ACC_module import ACCController, AccDecision


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


@dataclass
class LongDecision:
  button: Optional[int]
  log: str = ""


class LongController:
  def __init__(self) -> None:
    self.acc = ACCController()

  def update(self, CS, *, enabled: bool, now_ms: Optional[int] = None) -> LongDecision:
    now = _now_ms() if now_ms is None else int(now_ms)

    if not bool(getattr(CS, "enableACC", False)):
      return LongDecision(None, "gated: enableACC false")

    tinkla = getattr(CS, "_tinkla", None)
    if not bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False)):
      return LongDecision(None, "gated: adjust_acc_with_speed_limit false")

    speed_units = str(getattr(CS, "speed_units", "MPH"))

    try:
      desired_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      desired_ms = 0.0

    cruise_state = str(getattr(CS, "stock_cruise_state", ""))

    v_cruise_actual_kph = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0) * CV.MS_TO_KPH
    acc_speed_kph = float(desired_ms) * CV.MS_TO_KPH

    v_ego_ms = float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0)
    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    decision: AccDecision = self.acc.update(
      now_ms=now,
      enabled=bool(enabled),
      cruise_state=cruise_state,
      speed_units=speed_units,
      v_ego_ms=v_ego_ms,
      v_cruise_actual_kph=v_cruise_actual_kph,
      acc_speed_kph=acc_speed_kph,
      desired_speed_ms=desired_ms,
      cruise_buttons=cruise_buttons,
    )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, decision.reason)

    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    msg = (
      f"[XNOR_CRUISE_SYNC] uom={speed_units} "
      f"tgt={decision.target_kph*kph_to_u:.1f} "
      f"cur={decision.current_kph*kph_to_u:.1f} "
      f"btn={int(decision.button)} reason={decision.reason}"
    )
    cloudlog.info(msg)
    return LongDecision(int(decision.button), msg)
