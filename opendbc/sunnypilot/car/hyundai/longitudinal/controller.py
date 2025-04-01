import numpy as np
from dataclasses import dataclass

from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import CarControllerParams
from opendbc.sunnypilot.car.hyundai.longitudinal.tuning_controller import LongitudinalTuningController
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

STANDSTILL_DELAY = 0.9  # Delay in which commands from model are not sent


class JerkOutput:
  def __init__(self, jerk_upper, jerk_lower):
    self.jerk_upper = jerk_upper
    self.jerk_lower = jerk_lower


@dataclass
class LongitudinalState:
  accel: float = 0.0
  jerk_upper: float = 0.0
  jerk_lower: float = 0.0


class LongitudinalController:
  """Longitudinal controller which gets injected into CarControllerParams."""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP_SP = CP_SP
    self.tuning = LongitudinalTuningController(CP, CP_SP) if self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING else None
    self.long_state = LongitudinalState()
    self.force_zero = False
    self.last_stop_req_frame = 0  # Time when StopReq changed from 1 to 0 (note: StopReq uses stopping)

  def calculate_and_get_jerk(self, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    """Calculate jerk based on tuning."""
    self.tuning.make_jerk(CS, long_control_state)

    self.long_state.jerk_upper = self.tuning.jerk_upper
    self.long_state.jerk_lower = self.tuning.jerk_lower

  def calculate_accel(self, CC: structs.CarControl, CS: CarStateBase) -> None:
    """Calculate acceleration based on tuning and return the value."""
    self.long_state.accel = self.tuning.calculate_accel(CC, CS)

  def stopped_to_start_trans(self, long_control_state: LongCtrlState, frame: int) -> None:
    # Determine if zero acceleration should be forced
    if long_control_state == LongCtrlState.stopping:
      self.last_stop_req_frame = frame

    in_standstill_delay = (frame - self.last_stop_req_frame) * DT_CTRL < STANDSTILL_DELAY
    self.force_zero = self.tuning is not None and in_standstill_delay

  def update(self, CC: structs.CarControl, CS: CarStateBase, frame: int) -> None:
    """Inject Longitudinal Controls for HKG Vehicles."""
    actuators = CC.actuators
    long_control_state = actuators.longControlState

    self.calculate_and_get_jerk(CS, long_control_state)
    self.calculate_accel(CC, CS)
    self.stopped_to_start_trans(long_control_state, frame)

    if self.force_zero:
      # Force zero acceleration during standstill delay of 0.9 seconds
      self.long_state.accel = 0.0
      self.long_state.jerk_upper = 0.0
      self.long_state.jerk_lower = 0.0
