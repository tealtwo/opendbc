import numpy as np
from dataclasses import dataclass

from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import CarControllerParams
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_longitudinal_tune
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
  jerk: JerkOutput | None = None


class LongitudinalController:
  """Longitudinal controller which gets injected into CarControllerParams."""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP_SP = CP_SP
    self.tuning = LongitudinalTuningController(CP) if self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING else None
    self.state = LongitudinalState()
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0
    self.last_stop_req_frame = 0  # Time when StopReq changed from 1 to 0 (note: StopReq uses stopping)

  def get_jerk(self) -> JerkOutput:
    if self.tuning is not None:
      return JerkOutput(
        self.tuning.jerk_upper,
        self.tuning.jerk_lower,
      )
    else:
      return JerkOutput(
        self.jerk_upper,
        self.jerk_lower,
      )

  def calculate_and_get_jerk(self, CS: CarStateBase, long_control_state: LongCtrlState) -> JerkOutput:
    """Calculate jerk based on tuning and return JerkOutput."""
    if self.tuning is not None:
      self.tuning.make_jerk(CS)
    else:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = jerk_limit
    return self.get_jerk()

  def calculate_accel(self, CC: structs.CarControl, CS: CarStateBase, CP: structs.CarParams) -> float:
    """Calculate acceleration based on tuning and return the value."""
    if CP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING and self.tuning is not None:
      accel = self.tuning.calculate_accel(CC, CS)
    else:
      accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    return accel

  def update(self, CC: structs.CarControl, CS: CarStateBase, CP: structs.CarParams, frame: int) -> None:
    """Inject Longitudinal Controls for HKG Vehicles."""
    actuators = CC.actuators
    long_control_state = actuators.longControlState

    jerk_output = self.calculate_and_get_jerk(CS, long_control_state)
    self.state.jerk = jerk_output  # Store the JerkOutput object from our def.

    # Determine if zero acceleration should be forced
    if long_control_state == LongCtrlState.stopping:
      self.last_stop_req_frame = frame

    in_standstill_delay = (frame - self.last_stop_req_frame) * DT_CTRL < STANDSTILL_DELAY
    force_zero = self.tuning is not None and in_standstill_delay

    if force_zero:
      # Force zero acceleration during standstill delay of 0.9 seconds
      self.state.accel = 0.0
      self.jerk_upper = self.jerk_lower = 0.0
    else:
      # Not transitioning from stopping
      self.state.accel = self.calculate_accel(CC, CS, CP)
      self.jerk_upper = jerk_output.jerk_upper
      self.jerk_lower = jerk_output.jerk_lower
