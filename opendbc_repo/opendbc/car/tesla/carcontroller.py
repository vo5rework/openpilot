# /data/openpilot/opendbc/car/tesla/carcontroller.py
"""Tesla CarController (xnor C3)

Stable steering + Unity-parity virtual stalk for cruise speed-limit matching.

What this file does (only two things):
  1) publishes internal 0x659 (fake DAS) on bus 0 and bus 4 for panda safety (existing xnor behavior)
  2) when enabled + Tesla cruise is engaged, nudges Tesla cruise SET speed toward map speed limit
     by emitting STW_ACTN_RQ (cruise stalk up/down), using TeslaCAN.create_action_request() (CRC+counter).

It does *not* change steering behavior or ALC behavior.
"""

from __future__ import annotations

import numpy as np

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.tesla.teslacan import TeslaCAN
try:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANLegacy as TeslaCANLegacy
except ImportError:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven as TeslaCANLegacy

from opendbc.car.tesla.values import DBC, CarControllerParams, CANBUS, LEGACY_CARS, CAR

try:
  from opendbc.car.tesla.teslacan import create_fake_das_msg as create_fake_das
except ImportError:
  from opendbc.car.tesla.teslacan import create_fake_das_message as create_fake_das


# SpdCtrlLvr_Stat (STW_ACTN_RQ)
BTN_IDLE = 0
BTN_CANCEL = 1
BTN_MAIN = 2
BTN_UP2 = 4
BTN_DOWN2 = 8
BTN_UP1 = 16
BTN_DOWN1 = 32


def _resolve_dbc_name(dbc_names, CP, bus: Bus) -> str:
  """Resolve a DBC name for a given bus.

  XNOR note: interfaces.py passes dbc_names built from CarState.get_can_parsers().
  Some legacy builds omit Bus.party/Bus.pt keys; this must never crash card.
  """
  candidates = []
  if isinstance(dbc_names, dict):
    candidates.extend([bus, getattr(bus, "value", None), str(bus)])
    for k in candidates:
      if k is None:
        continue
      try:
        if k in dbc_names and dbc_names[k]:
          return dbc_names[k]
      except TypeError:
        continue

  # Try CP.dbc (Map(Text,Text)) if present
  cp_dbc = getattr(CP, "dbc", None)
  if isinstance(cp_dbc, dict):
    for k in (getattr(bus, "value", None), str(bus)):
      if k is None:
        continue
      if k in cp_dbc and cp_dbc[k]:
        return cp_dbc[k]

  # Final authoritative fallback: Tesla DBC map for this fingerprint
  try:
    dbc_map = DBC[CP.carFingerprint]
    if bus in dbc_map and dbc_map[bus]:
      return dbc_map[bus]
  except Exception:
    pass

  keys = list(dbc_names.keys()) if isinstance(dbc_names, dict) else type(dbc_names)
  raise KeyError(
    f"Missing DBC for bus={bus}. dbc_names keys={keys} "
    f"cp_dbc keys={list(cp_dbc.keys()) if isinstance(cp_dbc, dict) else None}"
  )


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, VM=None):
    try:
      super().__init__(dbc_names, CP, VM)
    except TypeError:
      super().__init__(dbc_names, CP)

    self.CP = CP
    self.frame = 0

    self.params = Params()
    self._cached_autopilot_disabled = False
    self._cached_pedal_enabled = False
    self._cached_adjust_acc_with_speed_limit = False
    self._cached_speed_limit_offset_uom = 0.0
    self._cached_speed_limit_use_relative = False
    self._params_last_read_frame = -100000

    self._op659_prev_btn = 0
    self.apply_angle_last = 0.0

    self._speed_sync_last_frame = -100000


    # Speed-limit sync (Unity-style pulse pacing)
    self._acc_target_u = 0
    self._acc_last_press_frame = -100000
    self._acc_last_current_u = 0
    self._acc_no_progress = 0
    self._stw_seed = None
    self._stw_seed_bus = int(CANBUS.party)
    self._stw_last_send_frame = -100000
    self._stw_release_frame = -1
    self._stw_release_bus = int(CANBUS.party)
    self._stw_sequence = []  # list[(frame:int, btn:int)]
    self._op_enabled_prev = False

    # Debounced "hold" state for stalk presses (repeat at ~10Hz, then release)
    self._stw_hold_btn = None  # type: int | None
    self._stw_hold_end_frame = -1
    self._stw_hold_next_frame = -1
    self._stw_hold_bus = int(CANBUS.party)

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {
        CANBUS.party: CANPacker(_resolve_dbc_name(dbc_names, CP, Bus.party)),
        CANBUS.powertrain: CANPacker(_resolve_dbc_name(dbc_names, CP, Bus.pt)),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)

      # STW_ACTN_RQ needs CRC/counter; legacy helper doesn't implement it.
      self._action_can_by_bus = {int(bus): TeslaCAN(pkr) for bus, pkr in self.packers.items()}
    else:
      self.packer = CANPacker(_resolve_dbc_name(dbc_names, CP, Bus.party))
      self.tesla_can = TeslaCAN(self.packer)
      self._action_can_by_bus = {int(CANBUS.party): self.tesla_can}

  def _refresh_cached_params(self) -> None:
    if (self.frame - self._params_last_read_frame) < 50:
      return
    self._params_last_read_frame = int(self.frame)

    self._cached_autopilot_disabled = bool(self.params.get_bool("TinklaAutopilotDisabled"))
    self._cached_pedal_enabled = bool(
      self.params.get_bool("TinklaPedalEnabled") or
      self.params.get_bool("PedalEnabled")
    )
    self._cached_adjust_acc_with_speed_limit = bool(self.params.get_bool("TinklaAdjustAccWithSpeedLimit"))
    self._cached_speed_limit_use_relative = bool(self.params.get_bool("TinklaSpeedLimitUseRelative"))
    try:
      self._cached_speed_limit_offset_uom = float(self.params.get("TinklaSpeedLimitOffset", encoding="utf-8") or "0")
    except Exception:
      self._cached_speed_limit_offset_uom = 0.0

  def _emit_internal_0x659(self, CS, can_sends) -> None:
    stalk_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    prev_btn = int(self._op659_prev_btn)

    main_edge = (stalk_btn == BTN_MAIN) and (prev_btn != BTN_MAIN)
    cancel_edge = (stalk_btn == BTN_CANCEL) and (prev_btn != BTN_CANCEL)

    self._op659_prev_btn = stalk_btn

    if (self.frame % 10 == 0) or main_edge or cancel_edge:
      buses = {int(CANBUS.party)}
      if self.CP.carFingerprint in LEGACY_CARS:
        buses.add(int(CANBUS.powertrain))
      for bus in sorted(buses):
        can_sends.append(create_fake_das(
          self._cached_pedal_enabled,
          self._cached_autopilot_disabled,
          bus=bus,
          stalk_main=main_edge,
          stalk_cancel=cancel_edge,
        ))

  def _speed_limit_target_ms(self, CS) -> float:
    # Prefer CarState's helper (uses DAS fused if present + supports relative offset)
    try:
      return float(CS._calc_speed_limit_target_ms(str(getattr(CS, "speed_units", "MPH"))))
    except Exception:
      pass

    limit_ms = float(getattr(CS, "speed_limit_ms_das", 0.0) or getattr(CS, "speed_limit_ms", 0.0) or 0.0)
    if limit_ms <= 0.0:
      return 0.0

    off = float(self._cached_speed_limit_offset_uom)
    if self._cached_speed_limit_use_relative:
      return max(0.0, limit_ms * (1.0 + off / 100.0))

    uom = str(getattr(CS, "speed_units", "MPH"))
    return max(0.0, limit_ms + (off * (CV.KPH_TO_MS if uom == "KPH" else CV.MPH_TO_MS)))

  def _stw_bus(self, CS) -> int:
    try:
      return int(getattr(CS, "stw_actn_bus", CANBUS.party))
    except Exception:
      return int(CANBUS.party)

  def _action_can_for_bus(self, bus: int):
    return (
      self._action_can_by_bus.get(int(bus)) or
      self._action_can_by_bus.get(int(CANBUS.party)) or
      next(iter(self._action_can_by_bus.values()))
    )


  def _send_stw(self, CS, can_sends, btn: int, *, bus: int | None = None) -> bool:
    """Send STW_ACTN_RQ seeded from the latest observed vehicle frame (Unity parity)."""
    msg = getattr(CS, "msg_stw_actn_req", None)
    if msg is None:
      return False

    b = int(bus if bus is not None else self._stw_bus(CS))
    seed = dict(msg)

    can_sends.append(self._action_can_for_bus(b).create_action_request(int(b), seed, int(btn)))

    self._stw_seed_bus = int(b)
    self._stw_last_send_frame = int(self.frame)
    return True
  def _queue_stalk_hold(self, CS, can_sends, btn: int, *, hold_frames: int = 35, interval_frames: int = 10) -> bool:
    """Debounced hold: repeat press at ~10Hz for ~0.35s, then release."""
    if self._stw_hold_btn is not None:
      return False
    if int(self._stw_release_frame) >= int(self.frame):
      return False
    if bool(self._stw_sequence):
      return False

    if not self._send_stw(CS, can_sends, btn):
      return False

    self._stw_hold_btn = int(btn)
    self._stw_hold_bus = int(self._stw_seed_bus)
    self._stw_hold_end_frame = int(self.frame) + int(hold_frames)
    self._stw_hold_next_frame = int(self.frame) + int(interval_frames)
    return True

  def _queue_stalk_pulse(self, CS, can_sends, btn: int) -> bool:
    # Legacy/sequence pulse: press now, release next frame.
    if int(self._stw_release_frame) > int(self.frame):
      return False

    if not self._send_stw(CS, can_sends, btn):
      return False

    self._stw_release_frame = int(self.frame) + 1
    self._stw_release_bus = int(self._stw_seed_bus)
    return True

  def _process_stalk_actions(self, CS, can_sends) -> None:
    # Hold-mode (debounced) press repeat + release
    if self._stw_hold_btn is not None:
      if int(self.frame) >= int(self._stw_hold_end_frame):
        self._send_stw(CS, can_sends, BTN_IDLE, bus=int(self._stw_hold_bus))
        self._stw_hold_btn = None
        self._stw_hold_end_frame = -1
        self._stw_hold_next_frame = -1
      elif int(self.frame) >= int(self._stw_hold_next_frame):
        self._send_stw(CS, can_sends, int(self._stw_hold_btn), bus=int(self._stw_hold_bus))
        self._stw_hold_next_frame = int(self._stw_hold_next_frame) + 10

    # Release pending pulse
    if int(self._stw_release_frame) == int(self.frame):
      self._send_stw(CS, can_sends, BTN_IDLE, bus=int(self._stw_release_bus))
      self._stw_release_frame = -1

    # Run queued press sequence (e.g. legacy MAIN+SET on engage)
    if (self._stw_hold_btn is None) and (int(self._stw_release_frame) < 0) and self._stw_sequence:
      due_frame, btn = self._stw_sequence[0]
      if int(self.frame) >= int(due_frame):
        if self._queue_stalk_pulse(CS, can_sends, int(btn)):
          self._stw_sequence.pop(0)


  def _speed_limit_sync(self, CC, CS, can_sends) -> None:
    # Only when OP is engaged and user enabled this feature.
    enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    if not enabled:
      self._acc_no_progress = 0
      return

    if not self._cached_autopilot_disabled:
      return

    if not self._cached_adjust_acc_with_speed_limit:
      return

    # Don't overlap with press/release sequencing or explicit sequences
    if (int(self._stw_release_frame) >= 0) or bool(self._stw_sequence):
      return

    if not bool(getattr(CS, "stock_cruise_enabled", False)):
      self._acc_no_progress = 0
      if (self.frame % 200) == 0:
        cloudlog.info("[XNOR_CRUISE_SYNC] gated: stock cruise not enabled")
      return

    target_ms = float(self._speed_limit_target_ms(CS))
    current_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)

    if target_ms <= 0.1 or current_ms <= 0.1:
      self._acc_no_progress = 0
      if (self.frame % 200) == 0:
        cloudlog.info(
          f"[XNOR_CRUISE_SYNC] gated: target_ms={target_ms:.2f} current_ms={current_ms:.2f} "
          f"speedLimit_ms={float(getattr(CS, 'speed_limit_ms', 0.0) or 0.0):.2f}"
        )
      return

    uom = str(getattr(CS, "speed_units", "MPH"))
    ms_to_u = CV.MS_TO_KPH if uom == "KPH" else CV.MS_TO_MPH

    target_u = int(round(target_ms * ms_to_u))
    current_u = int(round(current_ms * ms_to_u))
    if target_u <= 0 or current_u <= 0:
      self._acc_no_progress = 0
      return

    # Update target every cycle (map/sign can change); clamp to reasonable range
    self._acc_target_u = int(target_u)

    diff_u = int(self._acc_target_u - current_u)

    # Done (within 1 unit)
    if abs(diff_u) < 1:
      self._acc_no_progress = 0
      return

    # Conservative pulse pacing: 0.5s (Unity-style "slow nudge", avoids cruise faults)
    if (self.frame - int(self._acc_last_press_frame)) < 50:
      return

    # No-progress guard: if our reported set speed isn't moving, stop spamming presses.
    if current_u == int(self._acc_last_current_u):
      self._acc_no_progress = int(self._acc_no_progress) + 1
      if self._acc_no_progress >= 4:
        cloudlog.info(f"[XNOR_CRUISE_SYNC] abort: no progress current={current_u} target={self._acc_target_u}")
        self._acc_no_progress = 0
        self._acc_last_press_frame = int(self.frame)
        return
    else:
      self._acc_no_progress = 0

    # Choose 5-unit vs 1-unit press
    if diff_u > 0:
      btn = BTN_UP2 if diff_u >= 5 else BTN_UP1
    else:
      btn = BTN_DOWN2 if diff_u <= -5 else BTN_DOWN1

    if self._queue_stalk_pulse(CS, can_sends, btn):
      self._acc_last_press_frame = int(self.frame)
      self._acc_last_current_u = int(current_u)
      cloudlog.info(
        f"[XNOR_CRUISE_SYNC] uom={uom} target={float(self._acc_target_u):.1f} current={float(current_u):.1f} "
        f"diff={float(diff_u):.1f} btn={btn}"
      )
    else:
      if (self.frame % 200) == 0:
        cloudlog.info("[XNOR_CRUISE_SYNC] gated: missing CS.msg_stw_actn_req")
  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._refresh_cached_params()
    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = bool(self._cached_autopilot_disabled)

    # Always define before use
    human_control = bool(getattr(CS, "human_control", False))

    op_enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    if op_enabled and (not bool(self._op_enabled_prev)):
      if (autopilot_disabled and (self.CP.carFingerprint in LEGACY_CARS) and
          (float(getattr(CS.out, "vEgo", 0.0)) >= (18.0 * CV.MPH_TO_MS)) and
          (not bool(getattr(CS, "stock_cruise_enabled", False))) and
          (not bool(self._stw_sequence))):
        # Unity parity: legacy cars often require MAIN + SET on engage
        self._stw_sequence = [(int(self.frame), BTN_MAIN), (int(self.frame) + 10, BTN_DOWN1)]
        cloudlog.info("[XNOR_CRUISE_SYNC] legacy engage: queued MAIN+SET")
    self._op_enabled_prev = bool(op_enabled)

    self._process_stalk_actions(CS, can_sends)

    self._speed_limit_sync(CC, CS, can_sends)

    lat_active = (
      bool(CC.latActive) and
      autopilot_disabled and
      (not CS.out.cruiseState.standstill) and
      (not human_control)
    )

    # Steering (50Hz)
    if self.frame % 2 == 0:
      if human_control:
        self.apply_angle_last = float(CS.out.steeringAngleDeg)
      else:
        self.apply_angle_last = float(apply_std_steer_angle_limits(
          float(actuators.steeringAngleDeg),
          float(self.apply_angle_last),
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
          float(CS.out.steeringAngleDeg),
          lat_active,
          CarControllerParams.ANGLE_LIMITS,
        ))

      if self.CP.carFingerprint in LEGACY_CARS:
        counter = (self.frame // 2) % 16
        can_sends.append(
          self.tesla_can.create_steering_control(counter, self.apply_angle_last, lat_active)
        )
      else:
        can_sends.append(
          self.tesla_can.create_steering_control(self.apply_angle_last, lat_active)
        )

    # EPS allow (legacy)
    if (self.CP.carFingerprint in LEGACY_CARS) and (self.frame % 10 == 0):
      counter = (self.frame // 10) % 16
      can_sends.append(self.tesla_can.create_steering_allowed(counter))

    # Longitudinal (optional)
    if self.CP.openpilotLongitudinalControl and (self.frame % 4 == 0):
      state = 13 if CC.cruiseControl.cancel else 4
      accel = float(np.clip(
        float(actuators.accel),
        CarControllerParams.ACCEL_MIN,
        CarControllerParams.ACCEL_MAX
      ))
      counter = (self.frame // 4) % 8
      long_active = bool(CC.longActive) and (not autopilot_disabled)

      can_sends.append(
        self.tesla_can.create_longitudinal_command(
          state,
          accel,
          counter,
          float(CS.out.vEgo),
          long_active
        )
      )

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends


# ===== ABSTRACT-SAFETY SHIM =====
# If an indentation/merge slip moves CarController.update() outside the class,
# Python will treat CarController as abstract and crash at startup. This shim
# installs a minimal compatible update() only when required.
import abc as _abc  # noqa: E402
import inspect as _inspect  # noqa: E402

def _cc_update_shim(self, CC, CS, now_nanos, *args, **kwargs):  # noqa: D401
  actuators = getattr(CC, "actuators", None) or CC.actuators
  can_sends = kwargs.get("can_sends") or kwargs.get("can_sends_in") or []
  try:
    f = getattr(self, "_refresh_cached_params", None)
    if callable(f):
      f()
    f = getattr(self, "_emit_internal_0x659", None)
    if callable(f):
      f(CS, can_sends)
  except Exception:
    pass

  new_actuators = actuators.as_builder()
  try:
    new_actuators.steeringAngleDeg = float(getattr(self, "apply_angle_last", 0.0))
  except Exception:
    pass

  try:
    self.frame = int(getattr(self, "frame", 0)) + 1
  except Exception:
    pass
  return new_actuators, can_sends

if _inspect.isabstract(CarController):  # pragma: no cover
  try:
    cloudlog.error(f"[XNOR] CarController abstract ({sorted(getattr(CarController, '__abstractmethods__', []))}); applying shim")
  except Exception:
    pass
  CarController.update = _cc_update_shim
  _abc.update_abstractmethods(CarController)
# ===== END ABSTRACT-SAFETY SHIM =====