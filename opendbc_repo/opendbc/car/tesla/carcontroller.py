# /data/openpilot/opendbc/car/tesla/carcontroller.py
import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.tesla.teslacan import TeslaCAN
try:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANLegacy
except ImportError:  # pragma: no cover
  TeslaCANLegacy = None  # type: ignore

from opendbc.car.tesla.values import CANBUS, CAR, LEGACY_CARS, CarControllerParams, CruiseButtons
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)
    self.CCP = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.params = Params()
    self.frame = 0

    self.apply_angle_last = 0.0
    self._cached_pedal_enabled = False
    self._cached_autopilot_disabled = False
    self._cached_adjust_speed_limit = False
    self._cached_enable_alc = False

    if CP.carFingerprint in LEGACY_CARS:
      self.packers = {
        CANBUS.party: CANPacker(dbc_name),
        CANBUS.radar: CANPacker("tesla_powertrain"),
        CANBUS.autopilot_party: CANPacker(dbc_name),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)  # type: ignore[misc]
      # TeslaCANLegacy doesn't implement STW_ACTN_RQ button injection; use TeslaCAN for that.
      self.action_can = TeslaCAN(self.packers[CANBUS.party])
      self._action_buses = (CANBUS.party, CANBUS.autopilot_party)
    else:
      self.tesla_can = TeslaCAN(self.packer)
      self.action_can = self.tesla_can
      self._action_buses = (CANBUS.party,)

    # Virtual stalk injection state (Unity parity)
    self._stw_queue: list[tuple[int, int | None]] = []
    self._speed_sync_last_frame = -1_000_000
    self._last_human_cruise_action_frame = -1_000_000

    # ALC blinker hold (Unity parity behavior requested)
    self._blinker_hold_dir = 0  # 1-left, 2-right
    self._blinker_cancel_until_frame = 0
    self._prev_in_lane_change = False

  def _refresh_cached_params(self) -> None:
    # Params are expensive; refresh at 2Hz.
    if (self.frame % 50) != 0:
      return
    try:
      self._cached_pedal_enabled = bool(self.params.get_bool("TinklaEnableACC"))
      self._cached_autopilot_disabled = bool(self.params.get_bool("TinklaAutopilotDisabled"))
      self._cached_adjust_speed_limit = bool(self.params.get_bool("TinklaAdjustAccWithSpeedLimit"))
      self._cached_enable_alc = bool(self.params.get_bool("TinklaEnableALC"))
    except Exception:
      pass

  def _emit_internal_0x659(self, CS, can_sends: list) -> None:
    # Internal openpilot->panda msg (0x659). Safety consumes + blocks it from the car.
    if not self._cached_autopilot_disabled:
      return
    if (self.frame % 10) != 0:
      return
    can_sends.append(
      self.action_can.create_fake_das_msg(
        pedalEnabled=bool(self._cached_pedal_enabled),
        autopilot_disabled=bool(self._cached_autopilot_disabled),
        bus=CANBUS.party,
      )
    )
    # Legacy safety expects this also on bus+4.
    can_sends.append(
      self.action_can.create_fake_das_msg(
        pedalEnabled=bool(self._cached_pedal_enabled),
        autopilot_disabled=bool(self._cached_autopilot_disabled),
        bus=CANBUS.party + 4,
      )
    )

  def _enqueue_stw(self, cruise_button: int, *, turn_signal: int | None = None) -> None:
    # turn_signal: 1=left, 2=right, 3=neutral; None leaves current state unchanged.
    self._stw_queue.append((int(cruise_button), int(turn_signal) if turn_signal is not None else None))

  def _pulse_cruise_button(self, cruise_button: int) -> None:
    self._enqueue_stw(cruise_button)
    self._enqueue_stw(CruiseButtons.IDLE)

  def _send_one_stw(self, CS, can_sends: list) -> None:
    if not self._stw_queue:
      return
    if getattr(CS, "msg_stw_actn_req", None) is None:
      self._stw_queue.clear()
      return

    cruise_btn, turn_sig = self._stw_queue.pop(0)
    msg = dict(CS.msg_stw_actn_req or {})
    if turn_sig is not None:
      msg["TurnIndLvr_Stat"] = int(turn_sig)

    for bus in self._action_buses:
      can_sends.append(self.action_can.create_action_request(bus, msg, cruise_btn))

  def _update_alc_blinker_hold(self, CC, CS) -> None:
    if not (self._cached_enable_alc and bool(getattr(CS, "enableALC", False))):
      self._blinker_hold_dir = 0
      self._blinker_cancel_until_frame = 0
      self._prev_in_lane_change = False
      return

    lat_active = bool(getattr(CC, "latActive", False))
    if not lat_active:
      # Still allow a short neutral push to cancel after a completed lane change.
      if self._blinker_cancel_until_frame > 0 and (self.frame % 10) == 0:
        self._enqueue_stw(CruiseButtons.IDLE, turn_signal=3)
      if self._blinker_cancel_until_frame and self.frame >= self._blinker_cancel_until_frame:
        self._blinker_cancel_until_frame = 0
        self._blinker_hold_dir = 0
      self._prev_in_lane_change = False
      return

    # Never override when the driver is physically holding the stalk.
    if int(getattr(CS, "turnSignalStalkState", 0) or 0) in (1, 2):
      self._blinker_hold_dir = 0
      self._blinker_cancel_until_frame = 0
      self._prev_in_lane_change = False
      return

    in_lc = bool(
      getattr(CS, "alca_pre_engage", False)
      or getattr(CS, "alca_engaged", False)
      or getattr(CS, "alca_need_engagement", False)
    )

    if in_lc and not self._prev_in_lane_change:
      dir_ = int(getattr(CS, "alca_direction", 0) or 0)
      if dir_ not in (1, 2):
        if bool(getattr(CS, "leftBlinkerLamp", False)):
          dir_ = 1
        elif bool(getattr(CS, "rightBlinkerLamp", False)):
          dir_ = 2
        else:
          dir_ = int(getattr(CS, "tap_direction", 0) or 0)
      self._blinker_hold_dir = dir_ if dir_ in (1, 2) else 0
      self._blinker_cancel_until_frame = 0

    # Falling edge / completion: force stalk back to neutral briefly.
    if (not in_lc and self._prev_in_lane_change) or bool(getattr(CS, "alca_done", False)):
      if self._blinker_hold_dir in (1, 2):
        self._blinker_cancel_until_frame = self.frame + 30  # ~0.3s
      self._prev_in_lane_change = False
    else:
      self._prev_in_lane_change = in_lc

    if in_lc and self._blinker_hold_dir in (1, 2):
      if (self.frame % 10) == 0:  # 10Hz keepalive
        self._enqueue_stw(CruiseButtons.IDLE, turn_signal=self._blinker_hold_dir)
    elif self._blinker_cancel_until_frame > 0:
      if (self.frame % 10) == 0:
        self._enqueue_stw(CruiseButtons.IDLE, turn_signal=3)
      if self.frame >= self._blinker_cancel_until_frame:
        self._blinker_cancel_until_frame = 0
        self._blinker_hold_dir = 0

  def _update_speed_limit_sync(self, CC, CS) -> None:
    if not self._cached_adjust_speed_limit:
      return
    if not bool(getattr(CC, "enabled", False)):
      return
    if not bool(getattr(CS, "stock_cruise_enabled", False)):
      return

    # Respect recent human adjustments.
    human_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    if human_btn in (CruiseButtons.RES_ACCEL, CruiseButtons.RES_ACCEL_2ND, CruiseButtons.DECEL_SET, CruiseButtons.DECEL_2ND):
      self._last_human_cruise_action_frame = self.frame
    if (self.frame - self._last_human_cruise_action_frame) < 300:  # 3s
      return

    target_ms = 0.0
    try:
      target_ms = float(CS._calc_speed_limit_target_ms(getattr(CS, "speed_units", "MPH")))
    except Exception:
      target_ms = 0.0
    if target_ms <= 0.1:
      return

    current_ms = float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0)
    if current_ms <= 0.1:
      return

    units = getattr(CS, "speed_units", "MPH")
    ms_to_uom = CV.MS_TO_KPH if units == "KPH" else CV.MS_TO_MPH
    diff_uom = (target_ms - current_ms) * ms_to_uom

    # Match Unity ACC_module thresholds (0.9x of half-press, 0.6x of full-press).
    half_press = 1.0
    full_press = 5.0

    if abs(diff_uom) < 0.9 * half_press:
      return
    if (self.frame - self._speed_sync_last_frame) < 20:  # 5Hz max
      return
    self._speed_sync_last_frame = self.frame

    if diff_uom > 0:
      btn = CruiseButtons.RES_ACCEL_2ND if diff_uom >= 0.6 * full_press else CruiseButtons.RES_ACCEL
    else:
      btn = CruiseButtons.DECEL_2ND if diff_uom <= -0.6 * full_press else CruiseButtons.DECEL_SET

    self._pulse_cruise_button(btn)
    cloudlog.info(
      f"[CRUISE SYNC] target={target_ms*ms_to_uom:.1f} {units} current={current_ms*ms_to_uom:.1f} {units} btn={btn}"
    )

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends: list = []

    # Safety + param refresh
    self._refresh_cached_params()
    self._emit_internal_0x659(CS, can_sends)

    # Stalk injection (requires last STW_ACTN_RQ snapshot)
    if self._cached_autopilot_disabled and getattr(CS, "msg_stw_actn_req", None) is not None:
      self._update_alc_blinker_hold(CC, CS)
      self._update_speed_limit_sync(CC, CS)
      # Send at most one STW frame per cycle to avoid racing the real stalk.
      self._send_one_stw(CS, can_sends)
    else:
      self._stw_queue.clear()
      self._blinker_hold_dir = 0
      self._blinker_cancel_until_frame = 0
      self._prev_in_lane_change = False

    # Steering control (unchanged)
    lat_active = bool(CC.latActive)
    if (self.frame % self.CCP.STEER_STEP == 0) and (lat_active or (self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW1)):
      apply_angle = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg,
        self.apply_angle_last,
        CS.out.vEgo,
        self.CCP,
      )
      apply_angle = np.clip(apply_angle, CS.out.steeringAngleDeg - 20.0, CS.out.steeringAngleDeg + 20.0)
      can_sends.append(
        self.tesla_can.create_steering_control(
          apply_angle,
          lat_active and (not CS.human_control) and (not CS.out.cruiseState.standstill),
          0,
          CANBUS.party,
          1,
        )
      )
      self.apply_angle_last = apply_angle

    self.frame += 1
    return can_sends
