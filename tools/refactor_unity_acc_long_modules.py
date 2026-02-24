# /data/openpilot/tools/refactor_unity_acc_long_modules.py
"""
Refactor Tesla speed-limit ACC sync into Unity-style modules (ACC_module + LONG_module).

This script:
- writes /data/openpilot/selfdrive/car/modules/ACC_module.py
- writes /data/openpilot/selfdrive/car/modules/LONG_module.py
- patches Tesla CarController in BOTH trees:
    /data/openpilot/opendbc/car/tesla/carcontroller.py
    /data/openpilot/opendbc_repo/opendbc/car/tesla/carcontroller.py

Design notes:
- CRC/counter/bus selection stays in CarController via TeslaCAN.create_action_request + CS.msg_stw_actn_req + CS.stw_actn_bus.
- ACC/LONG modules ONLY decide which CruiseButtons press (Unity parity pacing).
- No steering logic changes.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


REPO_ROOT = Path("/data/openpilot")

MODULES_DIRS = [
  REPO_ROOT / "selfdrive" / "car" / "modules",
  REPO_ROOT / "openpilot" / "selfdrive" / "car" / "modules",
]

CC_TARGETS = [
  REPO_ROOT / "opendbc" / "car" / "tesla" / "carcontroller.py",
  REPO_ROOT / "opendbc_repo" / "opendbc" / "car" / "tesla" / "carcontroller.py",
]


ACC_MODULE_CODE = r'''"""
/data/openpilot/selfdrive/car/modules/ACC_module.py

Unity-parity ACC button selection + pacing (XNOR-friendly).

This module decides *which* Tesla cruise stalk button to emulate to move stock cruise
set speed toward a desired target speed.

It does NOT send CAN. It outputs a CruiseButtons value (or None).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _cc_units_kph(speed_units: str) -> tuple[float, float]:
  # Unity behavior: imperial cars adjust cruise in 1/5 mph, metric in 1/5 kph
  if speed_units == "MPH":
    return 1.0 * CV.MPH_TO_KPH, 5.0 * CV.MPH_TO_KPH
  return 1.0, 5.0


@dataclass
class AccDecision:
  button: Optional[int]
  reason: str
  target_kph: float = 0.0
  current_kph: float = 0.0
  est_kph: float = 0.0


class ACCController:
  # Unity constant: Tesla cruise only functions above ~17.1 mph
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

  def __init__(self) -> None:
    self.human_action_time_ms = 0
    self.automated_action_time_ms = 0
    self.prev_cruise_buttons = int(CruiseButtons.IDLE)

    # XNOR adaptation: set-speed readback can lag; keep a lightweight estimate to prevent oscillation.
    self._est_kph: Optional[float] = None
    self._est_time_ms: int = 0

  def note_human_buttons(self, cruise_buttons: int, *, now_ms: Optional[int] = None) -> None:
    """Call every update with the latest CS.cruise_buttons (raw)."""
    now = _now_ms() if now_ms is None else int(now_ms)
    btn = int(cruise_buttons or 0)
    if btn != int(CruiseButtons.IDLE) and self.prev_cruise_buttons == int(CruiseButtons.IDLE):
      self.human_action_time_ms = now
    self.prev_cruise_buttons = btn

  def _no_human_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return now_ms > int(self.human_action_time_ms) + int(milliseconds)

  def _no_automated_action_for(self, *, now_ms: int, milliseconds: int) -> bool:
    return now_ms > int(self.automated_action_time_ms) + int(milliseconds)

  def update(
    self,
    *,
    now_ms: int,
    enabled: bool,
    stock_cruise_enabled: bool,
    speed_units: str,
    v_ego_ms: float,
    current_set_speed_ms: float,
    desired_speed_ms: float,
    cruise_buttons: int,
  ) -> AccDecision:
    """
    Return the CruiseButtons int to press, or None.

    Unity parity:
    - blocks for 3s after human action
    - blocks for 400ms after automated action
    - CANCEL when target below min cruise OR when decel is very large
    """
    self.note_human_buttons(cruise_buttons, now_ms=now_ms)

    if not enabled:
      return AccDecision(None, "gated: not enabled")

    if not stock_cruise_enabled:
      return AccDecision(None, "gated: stock cruise not enabled")

    if not self._no_human_action_for(now_ms=now_ms, milliseconds=3000):
      return AccDecision(None, "gated: recent human action")

    if not self._no_automated_action_for(now_ms=now_ms, milliseconds=400):
      return AccDecision(None, "gated: cooldown")

    if desired_speed_ms <= 0.1 or current_set_speed_ms <= 0.1:
      return AccDecision(None, "gated: missing target/current")

    # Unity: do not try to adjust if below min cruise; CANCEL if target below min cruise
    if desired_speed_ms < self.MIN_CRUISE_SPEED_MS:
      self.automated_action_time_ms = now_ms
      self._est_kph = 0.0
      self._est_time_ms = now_ms
      return AccDecision(int(CruiseButtons.CANCEL), "cancel: target below min cruise")

    half_kph, full_kph = _cc_units_kph(speed_units)

    target_kph = float(desired_speed_ms) * CV.MS_TO_KPH
    readback_kph = float(current_set_speed_ms) * CV.MS_TO_KPH

    # XNOR adaptation: maintain est_kph to prevent repeated pulses when readback lags.
    if self._est_kph is None or (now_ms - int(self._est_time_ms)) > 2000:
      self._est_kph = readback_kph
      self._est_time_ms = now_ms
    else:
      if abs(readback_kph - float(self._est_kph)) <= (2.0 * full_kph):
        self._est_kph = readback_kph

    current_kph = float(self._est_kph)

    speed_offset_kph = target_kph - current_kph

    # Unity CANCEL guard for large decel requests
    if speed_offset_kph < (-2.0 * full_kph) and current_kph > 0.0:
      self.automated_action_time_ms = now_ms
      self._est_kph = 0.0
      self._est_time_ms = now_ms
      return AccDecision(int(CruiseButtons.CANCEL), "cancel: large decel", target_kph, readback_kph, current_kph)

    btn: Optional[int] = None

    # Reduce speed significantly
    if speed_offset_kph < (-0.6 * full_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_2ND)
    # Reduce slightly
    elif speed_offset_kph < (-0.9 * half_kph) and current_kph > 0.0:
      btn = int(CruiseButtons.DECEL_SET)
    # Increase speed only if car is above min cruise
    elif float(v_ego_ms) > float(self.MIN_CRUISE_SPEED_MS):
      available_kph = target_kph - current_kph
      if speed_offset_kph >= full_kph and full_kph < available_kph:
        btn = int(CruiseButtons.RES_ACCEL_2ND)
      elif speed_offset_kph >= half_kph and half_kph < available_kph:
        btn = int(CruiseButtons.RES_ACCEL)

    if btn is None:
      return AccDecision(None, "no-op", target_kph, readback_kph, current_kph)

    # Apply the estimate update immediately (prevents oscillation)
    if btn == int(CruiseButtons.RES_ACCEL_2ND):
      self._est_kph = current_kph + full_kph
    elif btn == int(CruiseButtons.RES_ACCEL):
      self._est_kph = current_kph + half_kph
    elif btn == int(CruiseButtons.DECEL_2ND):
      self._est_kph = max(0.0, current_kph - full_kph)
    elif btn == int(CruiseButtons.DECEL_SET):
      self._est_kph = max(0.0, current_kph - half_kph)
    elif btn == int(CruiseButtons.CANCEL):
      self._est_kph = 0.0

    self._est_time_ms = now_ms
    self.automated_action_time_ms = now_ms

    return AccDecision(btn, "press", target_kph, readback_kph, float(self._est_kph or 0.0))
'''


LONG_MODULE_CODE = r'''"""
/data/openpilot/selfdrive/car/modules/LONG_module.py

Unity-style LONG module wrapper (XNOR-friendly).

Today: only ACC speed-limit syncing.
Future: radar / follow distance / resume logic hooks.
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
    """
    Returns a cruise button to emulate (CruiseButtons int), or None.
    """
    now = _now_ms() if now_ms is None else int(now_ms)

    # Feature flags live in CarState (xnor CFG module pattern)
    if not bool(getattr(CS, "enableACC", False)):
      return LongDecision(None, "gated: enableACC false")

    tinkla = getattr(CS, "_tinkla", None)
    if not bool(tinkla and getattr(tinkla, "adjust_acc_with_speed_limit", False)):
      return LongDecision(None, "gated: adjust_acc_with_speed_limit false")

    speed_units = str(getattr(CS, "speed_units", "MPH"))

    # Desired speed comes from CarState helper (speed limit + offset logic)
    try:
      desired_ms = float(CS._calc_speed_limit_target_ms(speed_units))
    except Exception:
      desired_ms = 0.0

    # Stock set speed readback (XNOR port field)
    current_set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    stock_enabled = bool(getattr(CS, "stock_cruise_enabled", False))

    v_ego_ms = float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0)

    cruise_buttons = int(getattr(CS, "cruise_buttons", int(CruiseButtons.IDLE)) or 0)

    decision: AccDecision = self.acc.update(
      now_ms=now,
      enabled=bool(enabled),
      stock_cruise_enabled=stock_enabled,
      speed_units=speed_units,
      v_ego_ms=v_ego_ms,
      current_set_speed_ms=current_set_ms,
      desired_speed_ms=desired_ms,
      cruise_buttons=cruise_buttons,
    )

    if decision.button is None or int(decision.button) == int(CruiseButtons.IDLE):
      return LongDecision(None, decision.reason)

    # Keep logs short but informative
    kph_to_u = CV.KPH_TO_MPH if speed_units == "MPH" else 1.0
    msg = (
      f"[XNOR_CRUISE_SYNC] uom={speed_units} "
      f"tgt={decision.target_kph*kph_to_u:.1f} "
      f"cur={decision.current_kph*kph_to_u:.1f} "
      f"est={decision.est_kph*kph_to_u:.1f} "
      f"btn={int(decision.button)} reason={decision.reason}"
    )
    cloudlog.info(msg)
    return LongDecision(int(decision.button), msg)
'''


def _write_file(path: Path, content: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _backup(path: Path, stamp: str) -> Path:
  bak = path.with_suffix(path.suffix + f".bak_{stamp}")
  bak.write_text(path.read_text(errors="ignore"), encoding="utf-8")
  return bak


def _patch_carcontroller(src: str) -> str:
  """
  Patch:
  - import LongController
  - init self._long_module
  - _send_stw uses latest seed each send
  - _speed_limit_sync delegates to LongController + queues one pulse
  """

  # 1) ensure import
  if "from openpilot.selfdrive.car.modules.LONG_module import LongController" not in src:
    # place after existing module imports if present, else after swaglog import
    anchor = "from openpilot.common.swaglog import cloudlog\n"
    if anchor in src:
      src = src.replace(anchor, anchor + "from openpilot.selfdrive.car.modules.LONG_module import LongController\n", 1)
    else:
      src = "from openpilot.selfdrive.car.modules.LONG_module import LongController\n" + src

  # 2) init module in __init__
  if "self._long_module = LongController()" not in src:
    # insert after self.params = Params() (common in your file)
    m = re.search(r"(self\.params\s*=\s*Params\(\)\s*\n)", src)
    if not m:
      raise RuntimeError("Could not find 'self.params = Params()' in carcontroller.py")
    insert = "    self._long_module = LongController()\n"
    src = src[:m.end()] + insert + src[m.end():]

  # 3) replace _send_stw body to always seed from latest observed message
  send_pat = re.compile(r"\n\s*def _send_stw\(self, CS, can_sends, btn: int, \*, bus: int \| None = None\) -> bool:\n.*?\n\s*def _queue_stalk_pulse", re.S)
  m = send_pat.search(src)
  if not m:
    raise RuntimeError("Could not locate _send_stw(...) block in carcontroller.py")

  send_repl = """
  def _send_stw(self, CS, can_sends, btn: int, *, bus: int | None = None) -> bool:
    msg = getattr(CS, "msg_stw_actn_req", None)
    if msg is None:
      return False

    b = int(bus if bus is not None else self._stw_bus(CS))
    seed = dict(msg)  # Unity parity: seed from latest observed frame every send

    can_sends.append(self._action_can_for_bus(b).create_action_request(int(b), seed, int(btn)))
    self._stw_seed_bus = int(b)
    self._stw_last_send_frame = int(self.frame)
    return True

  def _queue_stalk_pulse"""
  src = src[:m.start()] + send_repl + src[m.end():]

  # 4) replace _speed_limit_sync with a delegator to LONG module (Unity-style pacing inside)
  sl_pat = re.compile(r"\n\s*def _speed_limit_sync\(self, CC, CS, can_sends\) -> None:\n.*?\n\s*def update\(self, CC, CS, now_nanos\):", re.S)
  m = sl_pat.search(src)
  if not m:
    raise RuntimeError("Could not locate _speed_limit_sync(...) block in carcontroller.py")

  sl_repl = r"""
  def _speed_limit_sync(self, CC, CS, can_sends) -> None:
    enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    if (not enabled) or (not self._cached_autopilot_disabled):
      return

    # Don't overlap with explicit sequences or a pending pulse release.
    if (int(self._stw_release_frame) >= 0) or bool(self._stw_sequence):
      return

    decision = self._long_module.update(CS, enabled=enabled, now_ms=int(self._now_ms()))
    if decision.button is None:
      return

    # One Unity-style pulse; release is handled next frame by _queue_stalk_pulse().
    if self._queue_stalk_pulse(CS, can_sends, int(decision.button)):
      self._automated_cruise_action_time_ms = int(self._now_ms())

  def update(self, CC, CS, now_nanos):
"""
  src = src[:m.start()] + sl_repl + src[m.end():]

  return src


def main() -> None:
  stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")

  # 1) write modules (both possible module roots)
  wrote_any = False
  for d in MODULES_DIRS:
    if d.exists():
      _write_file(d / "ACC_module.py", ACC_MODULE_CODE)
      _write_file(d / "LONG_module.py", LONG_MODULE_CODE)
      wrote_any = True

  if not wrote_any:
    # default to canonical path
    d = MODULES_DIRS[0]
    _write_file(d / "ACC_module.py", ACC_MODULE_CODE)
    _write_file(d / "LONG_module.py", LONG_MODULE_CODE)

  # 2) patch carcontroller files
  for p in CC_TARGETS:
    if not p.exists():
      raise RuntimeError(f"Missing target: {p}")
    orig = p.read_text(errors="ignore")
    bak = _backup(p, stamp)
    patched = _patch_carcontroller(orig)
    p.write_text(patched, encoding="utf-8")
    print(f"patched: {p} (backup: {bak})")

  print("DONE. Now run:")
  print("  python3 -m py_compile /data/openpilot/opendbc/car/tesla/carcontroller.py")
  print("  python3 -m py_compile /data/openpilot/opendbc_repo/opendbc/car/tesla/carcontroller.py")
  print("  sudo reboot")


if __name__ == "__main__":
  main()
