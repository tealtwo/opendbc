import numpy as np
from dataclasses import dataclass

from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams

from opendbc.sunnypilot.car.hyundai.longitudinal_config import Cartuning
from opendbc.sunnypilot.interpolation_utils import catmull_rom_interp

LongCtrlState = structs.CarControl.Actuators.LongControlState


@dataclass
class LongitudinalTuningState:
  accel_last: float = 0.0
  accel_last_jerk: float = 0.0
  jerk: float = 0.0


class LongitudinalTuningController:
  """Longitudinal tuning methodology for HKG"""

  def __init__(self, CP: structs.CarParams) -> None:
    self.CP = CP
    self.state = LongitudinalTuningState()
    self.car_config = Cartuning.get_car_config(CP)
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0
    self.last_decel_time = 0.0
    self.min_cancel_delay = 0.1

  def reset_jerk(self) -> None:
    self.state.accel_last_jerk = 0.0
    self.state.jerk = 0.0
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0

  def make_jerk(self, CS: CarStateBase) -> None:
    # Jerk is calculated using current accel - last accel divided by ΔT (delta time)
    current_accel = CS.out.aEgo
    self.state.jerk = (current_accel - self.state.accel_last_jerk) / 0.05  # DT_MDL == driving model which equals 0.05
    self.state.accel_last_jerk = current_accel

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    if velocity < 5.0:
      decel_jerk_max = self.car_config.jerk_limits[1]
    elif velocity > 20.0:
      decel_jerk_max = 2.5
    else:
      decel_jerk_max = 5.83 - (velocity / 6)

    accel_jerk_max = self.car_config.jerk_limits[2] if LongCtrlState == LongCtrlState.pid else 1.0
    jerk_lower_multiplier = 4.0 if self.CP.flags & HyundaiFlags.CANFD else 2.0
    self.jerk_upper = min(max(self.car_config.jerk_limits[0], self.state.jerk * 2.0), accel_jerk_max)
    self.jerk_lower = min(max(self.car_config.jerk_limits[0], -self.state.jerk * jerk_lower_multiplier), decel_jerk_max)

  def calculate_limited_accel(self, CC: structs.CarControl, CS: CarStateBase) -> float:
    """Adaptive acceleration limiting."""
    actuators = CC.actuators
    self.make_jerk(CS)
    target_accel = actuators.accel

    # Normal operation = above 17 m/s
    if CS.out.vEgo > 17.0 and target_accel < 0.01:
      brake_ratio = np.clip(abs(target_accel / self.car_config.accel_limits[0]), 0.0, 1.0)
      # Array comes from longitudinal_config.py, 1.0 = -3.5 accel, which will never be less than -3.5 EVER
      accel_rate_down = DT_CTRL * catmull_rom_interp(brake_ratio,
                                                     np.array([0.25, 0.5, 0.75, 1.0]),
                                                     np.array(self.car_config.brake_response))
      accel = max(target_accel, self.state.accel_last - accel_rate_down)
    else:
      accel = actuators.accel

    target_accel = accel + (target_accel - self.state.accel_last)
    accel = target_accel
    self.state.accel_last = accel
    return accel

  def calculate_accel(self, CC: structs.CarControl, CS: CarStateBase) -> float:
    """Calculate acceleration with cruise control status handling."""
    if not CC.enabled:
      self.reset_jerk()
      self.state.accel_last = 0.0
      return 0.0

    accel = self.calculate_limited_accel(CC, CS)
    return float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

  def apply_tune(self, CP: structs.CarParams) -> None:
    config = self.car_config
    CP.vEgoStopping = config.vego_stopping
    CP.vEgoStarting = config.vego_starting
    CP.stoppingDecelRate = config.stopping_decel_rate
    CP.startAccel = config.start_accel
    CP.startingState = True
    CP.longitudinalActuatorDelay = 0.5
