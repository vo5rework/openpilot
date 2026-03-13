"""/data/openpilot/selfdrive/car/modules/BLNK_module.py

Unity-parity "tap blinker" detection for Tesla ALC.

- Short stalk taps are recognized as comfort blinks.
- Tap ownership persists through active auto lane change.
- Opposite-direction stalk input cancels the latched tap immediately.
"""

from __future__ import annotations


class BLNKController:
  def __init__(self, tap_detect_frames: int = 55) -> None:
    self.tap_duration_frames = int(tap_detect_frames)
    self.tap_direction = 0
    self.blinker_on_frame_start = 0
    self.blinker_on_frame_end = 0
    self.prev_turnSignalStalkState = 0

  def _reset(self) -> None:
    self.tap_direction = 0
    self.blinker_on_frame_start = 0
    self.blinker_on_frame_end = 0

  def update_state(self, CS, frame: int) -> None:
    stalk_state = int(getattr(CS, "turnSignalStalkState", 0) or 0)

    if self.tap_direction > 0 and stalk_state > 0 and self.tap_direction != stalk_state:
      self._reset()

    if stalk_state > 0 and self.prev_turnSignalStalkState == 0:
      self.blinker_on_frame_start = int(frame)
    elif stalk_state == 0 and self.prev_turnSignalStalkState > 0:
      if int(frame) - int(self.blinker_on_frame_start) <= self.tap_duration_frames:
        self.tap_direction = int(self.prev_turnSignalStalkState)
        self.blinker_on_frame_end = int(frame)
      else:
        self._reset()

    if (
      self.tap_direction > 0 and
      (int(frame) - int(self.blinker_on_frame_start) > self.tap_duration_frames) and
      (int(frame) - int(self.blinker_on_frame_end) > self.tap_duration_frames) and
      int(getattr(CS, "alca_direction", 0) or 0) != int(self.tap_direction)
    ):
      self._reset()

    self.prev_turnSignalStalkState = stalk_state
