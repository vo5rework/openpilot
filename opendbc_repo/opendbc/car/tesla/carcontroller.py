# /data/openpilot/opendbc/car/tesla/carcontroller.py
"""Tesla CarController (xnor architecture).

This file keeps the existing steering + internal 0x659 publishing behavior intact and adds
two Unity-parity helpers implemented *safely* for xnor:

1) ALC blinker hold (tap-to-lane-change parity):
   - When xnor/Unity latches a tap blinker for ALC (`CS._alc_tap_latch_*`) or a lane change is active,
     we keep the physical blinker on by re-sending STW_ACTN_RQ with TurnIndLvr_Stat on the *same
     physical CAN bus where STW_ACTN_RQ was observed* (prevents HUD faults from wrong-bus injection).
   - When lane change finishes, we send a short neutral (3) to cancel.

2) Tesla cruise speed-limit sync (Unity parity):
   - Uses Tesla map/DAS speed limit from CarState (`CS._calc_speed_limit_target_ms`) and the Tesla
     cruise setpoint from CarState (`CS.stock_cruise_set_speed_ms`).
   - Nudges the setpoint with virtual stalk presses via STW_ACTN_RQ (UP/DOWN 1 or 5 units).

All STW injection is rate-limited and shares the same CRC/counter logic as the stock message via
TeslaCAN.create_action_request().
"""

from __future__ import annotations

import numpy as np

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

from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR
from openpilot.common.params import Params

try:
  from openpilot.common.swaglog import cloudlog
except Exception:  # pragma: no cover
  import logging
  cloudlog = logging.getLogger("carcontroller")

try:
  from opendbc.car.tesla.teslacan import create_fake_das_msg as create_fake_das
except ImportError:
  from opendbc.car.tesla.teslacan import create_fake_das_message as create_fake_das


# STW_ACTN_RQ.SpdCtrlLvr_Stat values (tesla_can.dbc)
_UP_1 = 16
_UP_5 = 4
_DN_1 = 32
_DN_5 = 8


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, VM=None):
    super().__init__(dbc_names, CP)

    self.CP = CP
    self.frame = 0

    self.params = Params()
    self._cached_autopilot_disabled = False
    self._cached_pedal_enabled = False
    self._params_last_read_frame = -100000

    # 0x659 (Unity parity) edge detection
    self._op659_prev_btn = 0

    # steering state
    self.apply_angle_last = 0.0

    # STW injection state
    self._stw_pending_release = False
    self._stw_release_frame = 0
    self._stw_last_cmd_frame = -100000
    self._stw_last_log_frame = -100000

    # blinker hold / cancel state
    self._prev_alca_done = False
    self._turn_cancel_until_frame = -1

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {
        CANBUS.party: CANPacker(dbc_names[Bus.party]),
        CANBUS.powertrain: CANPacker(dbc_names[Bus.pt]),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)
      # TeslaCANLegacy doesn't implement STW_ACTN_RQ; use TeslaCAN with the party packer.
      self.action_can = TeslaCAN(self.packers[CANBUS.party])
    else:
      self.packer = CANPacker(dbc_names[Bus.party])
      self.tesla_can = TeslaCAN(self.packer)
      self.action_can = self.tesla_can

  # ---------- internal helpers ----------

  def _refresh_cached_params(self) -> None:
    if (self.frame - self._params_last_read_frame) >= 50:
      self._params_last_read_frame = self.frame
      self._cached_autopilot_disabled = bool(self.params.get_bool("TinklaAutopilotDisabled"))
      self._cached_pedal_enabled = bool(
        self.params.get_bool("TinklaPedalEnabled") or
        self.params.get_bool("PedalEnabled")
      )

  def _emit_internal_0x659(self, CS, can_sends) -> None:
    stalk_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    prev_btn = int(self._op659_prev_btn)

    main_edge = (stalk_btn == 2) and (prev_btn != 2)
    cancel_edge = (stalk_btn == 1) and (prev_btn != 1)

    self._op659_prev_btn = stalk_btn

    if (self.frame % 10 == 0) or main_edge or cancel_edge:
      # Keep original behavior: publish to both panda channels used by xnor (bus and bus+4).
      for bus in (CANBUS.party, CANBUS.party + 4):
        can_sends.append(create_fake_das(
          self._cached_pedal_enabled,
          self._cached_autopilot_disabled,
          bus=bus,
          stalk_main=main_edge,
          stalk_cancel=cancel_edge,
        ))

  def _stw_bus(self, CS) -> int:
    # CarState sets this to the *physical* CAN bus where STW_ACTN_RQ was observed.
    return int(getattr(CS, "stw_actn_bus", CANBUS.party))

  def _stw_base(self, CS) -> dict | None:
    msg = getattr(CS, "msg_stw_actn_req", None)
    return dict(msg) if isinstance(msg, dict) else None

  def _stw_send(self, CS, can_sends, *, cruise_button: int, turn_raw: int | None) -> None:
    base = self._stw_base(CS)
    if base is None:
      return

    if turn_raw is not None:
      base["TurnIndLvr_Stat"] = int(turn_raw)

    bus = self._stw_bus(CS)
    can_sends.append(self.action_can.create_action_request(bus, base, int(cruise_button)))
    self._stw_last_cmd_frame = self.frame

  def _stw_pulse(self, CS, can_sends, *, cruise_button: int, turn_raw: int | None) -> None:
    # press now, release next frame
    if self._stw_pending_release:
      return
    self._stw_send(CS, can_sends, cruise_button=int(cruise_button), turn_raw=turn_raw)
    self._stw_pending_release = True
    self._stw_release_frame = self.frame + 1

  def _stw_release_if_due(self, CS, can_sends, *, turn_raw: int | None) -> bool:
    if self._stw_pending_release and self.frame >= self._stw_release_frame:
      self._stw_send(CS, can_sends, cruise_button=0, turn_raw=turn_raw)
      self._stw_pending_release = False
      return True
    return False

  # ---------- blinker hold (Unity parity) ----------

  def _turn_override_raw(self, CS) -> int | None:
    # Don't fight the driver holding the stalk.
    if int(getattr(CS, "turnSignalStalkState", 0) or 0) in (1, 2):
      self._turn_cancel_until_frame = -1
      return None

    # Detect LC finish edge to cancel blinker shortly after.
    alca_done = bool(getattr(CS, "alca_done", False))
    if alca_done and not self._prev_alca_done:
      self._turn_cancel_until_frame = self.frame + 30  # ~0.3s at 100Hz
    self._prev_alca_done = alca_done

    if self._turn_cancel_until_frame >= 0 and self.frame <= self._turn_cancel_until_frame:
      return 3  # neutral/cancel
    if self._turn_cancel_until_frame >= 0 and self.frame > self._turn_cancel_until_frame:
      self._turn_cancel_until_frame = -1

    # Prefer lane-change direction if available; else hold the tap-latched dir.
    lc_dir = int(getattr(CS, "alca_direction", 0) or 0)
    latch_dir = int(getattr(CS, "_alc_tap_latch_dir", 0) or 0)

    if lc_dir in (1, 2) and (bool(getattr(CS, "alca_pre_engage", False)) or bool(getattr(CS, "alca_engaged", False))):
      return lc_dir

    # Latch active window comes from CarState and is in its frame domain; it increments every update.
    cs_frame = int(getattr(CS, "_param_frame", self.frame))
    latch_until = int(getattr(CS, "_alc_tap_latch_until", 0) or 0)
    if latch_dir in (1, 2) and latch_until > cs_frame:
      return latch_dir

    return None

  def _update_blinker_hold(self, CS, can_sends, lat_active: bool, turn_raw: int | None) -> None:
    if not lat_active or not bool(getattr(CS, "enableALC", False)):
      return
    if turn_raw is None:
      return

    # Keep alive at 20Hz. Any cruise-sync pulse will also include turn_raw, so skip if we just sent.
    if (self.frame % 5 == 0) and ((self.frame - self._stw_last_cmd_frame) > 0):
      self._stw_send(CS, can_sends, cruise_button=0, turn_raw=int(turn_raw))

  # ---------- speed limit ACC sync (Unity parity) ----------

  def _acc_sync_enabled(self, CC, CS, *, enabled: bool, autopilot_disabled: bool) -> bool:
    if not enabled or not autopilot_disabled:
      return False

    cfg = getattr(CS, "_tinkla", None)
    if cfg is not None and not bool(getattr(cfg, "adjust_acc_with_speed_limit", False)):
      return False
    if cfg is None and not bool(self.params.get_bool("TinklaAdjustAccWithSpeedLimit")):
      return False

    if not bool(getattr(CS, "stock_cruise_enabled", False)):
      return False

    if bool(getattr(CS.out, "gasPressed", False)) or bool(getattr(CS.out, "brakePressed", False)):
      return False

    # Don't fight the driver actively pressing the cruise stalk.
    if int(getattr(CS, "cruise_buttons", 0) or 0) != 0:
      return False

    return True

  def _acc_sync_target_ms(self, CS) -> float:
    try:
      return float(CS._calc_speed_limit_target_ms(str(getattr(CS, "speed_units", "MPH"))))
    except Exception:
      return float(getattr(CS, "speed_limit_ms", 0.0) or 0.0)

  def _acc_sync_update(self, CC, CS, can_sends, *, enabled: bool, autopilot_disabled: bool, turn_raw: int | None) -> None:
    if not self._acc_sync_enabled(CC, CS, enabled=enabled, autopilot_disabled=autopilot_disabled):
      return

    # run at ~2Hz
    if (self.frame - self._stw_last_cmd_frame) < 20:
      return

    target_ms = self._acc_sync_target_ms(CS)
    set_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)

    if target_ms <= 0.1 or set_ms <= 0.1:
      return

    units = str(getattr(CS, "speed_units", "MPH"))
    ms_to_u = CV.MS_TO_KPH if units == "KPH" else CV.MS_TO_MPH
    diff_u = (target_ms - set_ms) * ms_to_u

    if abs(diff_u) < 0.5:
      return

    # Choose 5-unit when far, else 1-unit.
    if abs(diff_u) >= 4.5:
      btn = _UP_5 if diff_u > 0 else _DN_5
    else:
      btn = _UP_1 if diff_u > 0 else _DN_1

    self._stw_pulse(CS, can_sends, cruise_button=int(btn), turn_raw=turn_raw)

    # Rate-limited log (~1Hz max)
    if (self.frame - self._stw_last_log_frame) >= 100:
      self._stw_last_log_frame = self.frame
      cloudlog.info(
        f"[ACC_SYNC] bus={self._stw_bus(CS)} target={target_ms*ms_to_u:.1f} set={set_ms*ms_to_u:.1f} diff={diff_u:+.1f} btn={btn}"
      )

  # ---------- main update ----------

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self._refresh_cached_params()
    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = bool(self._cached_autopilot_disabled)
    human_control = bool(getattr(CS, "human_control", False))

    lat_active = (
      bool(CC.latActive) and
      autopilot_disabled and
      (not CS.out.cruiseState.standstill) and
      (not human_control)
    )

    # Compute turn override once; reuse for both hold + cruise-sync pulses.
    turn_raw = self._turn_override_raw(CS)

    # 1) release any pending stalk pulse
    self._stw_release_if_due(CS, can_sends, turn_raw=turn_raw)

    # 2) speed-limit sync pulses (press scheduled, release next frame)
    self._acc_sync_update(CC, CS, can_sends, enabled=bool(CC.enabled), autopilot_disabled=autopilot_disabled, turn_raw=turn_raw)

    # 3) blinker keep-alive (20Hz)
    self._update_blinker_hold(CS, can_sends, lat_active=lat_active, turn_raw=turn_raw)

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
