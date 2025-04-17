"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from dataclasses import dataclass

from opendbc.car import structs, DT_CTRL, rate_limit
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase

from opendbc.car.hyundai.values import CarControllerParams
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import get_car_config
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

JERK_STEP = 0.1
JERK_THRESHOLD = 0.1


def jerk_limited_integrator(desired_accel, last_accel, jerk_upper, jerk_lower) -> float:
  if desired_accel >= last_accel:
    val = jerk_upper * DT_CTRL * 2
  else:
    val = jerk_lower * DT_CTRL * 2

  return rate_limit(desired_accel, last_accel, -val, val)


def ramp_update(current, target):
  if abs(target - current) > JERK_THRESHOLD:
    return current + float(np.clip(target - current, -JERK_STEP, JERK_STEP))
  return current


@dataclass
class LongitudinalTuningState:
  accel_last: float = 0.0
  jerk: float = 0.0


class LongitudinalTuningController:
  """Longitudinal tuning methodology for HKG"""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP

    self.state = LongitudinalTuningState()
    self.car_config = get_car_config(CP)
    self.accel_filter = FirstOrderFilter(0.0, 0.25, DT_CTRL * 2)
    self.desired_accel = 0.0
    self.actual_accel = 0.0
    self.jerk_upper = 0.5
    self.jerk_lower = 0.5
    self.stopping = False
    self.stopping_count = 0
    self.long_control_state_last = LongCtrlState.off

  def get_stopping_state(self, long_control_state: LongCtrlState) -> None:
    stopping = long_control_state == LongCtrlState.stopping
    self.long_control_state_last = long_control_state

    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      self.stopping = stopping
      self.stopping_count = 0
      return

    if not stopping:
      self.stopping = False
      self.stopping_count = 0
      return

    # 1 second after entering stopping state or when the last state was off
    if self.stopping_count > 1 / (DT_CTRL * 2) or self.long_control_state_last == LongCtrlState.off:
      self.stopping = True

    self.stopping_count += 1

  def calculate_a_value(self, CC: structs.CarControl) -> tuple[float, float]:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      self.desired_accel = CC.actuators.accel
      self.actual_accel = CC.actuators.accel
      return self.desired_accel, self.actual_accel

    if not CC.longActive:
      self.desired_accel = 0.0
      self.actual_accel = 0.0
      self.state.accel_last = 0.0
      return self.desired_accel, self.actual_accel

    # Force zero aReqRaw during StopReq
    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel

    return self.desired_accel, self.state.accel_last

  def calculate_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING_BRAKING:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = 5.0
      return

    # Apply acceleration filter
    self.desired_accel = CC.actuators.accel
    prev_filtered_accel = self.accel_filter.x
    self.accel_filter.update(self.desired_accel)
    filtered_accel = self.accel_filter.x

    # Calculate jerk
    self.state.jerk = (filtered_accel - prev_filtered_accel) / (DT_CTRL * 2)

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 5.0, 2.5]))

    planned_accel, previous_accel = self.calculate_a_value(CC)
    accel_error = planned_accel - previous_accel
    interp_error = min(accel_error, -0.001)

    lower_jerk = 5.0 if self.CP.radarUnavailable else (
      float(np.interp(interp_error, [-0.001, -0.0025, -0.005, -0.03, -0.25, -1.5],
                                    [1.0, 1.35, 2.0, 2.5, 3.3, 5.0]))
      if (accel_error <= -0.001 or self.desired_accel < -0.001) else 0.5
    )

    accel_jerk_max = self.car_config.jerk_limits[2] if long_control_state == LongCtrlState.pid else 1.0
    min_upper_jerk = 0.5 if (velocity > 3.0) else 0.725

    desired_jerk_upper = min(max(min_upper_jerk, self.state.jerk), accel_jerk_max)
    desired_jerk_lower = min(lower_jerk, speed_factor)

    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
    self.jerk_lower = ramp_update(self.jerk_lower, desired_jerk_lower)
