"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from dataclasses import dataclass

from opendbc.car import structs, DT_CTRL, rate_limit
from opendbc.car.interfaces import CarStateBase

from opendbc.car.hyundai.values import CarControllerParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState

JERK_STEP = 0.1
JERK_THRESHOLD = 0.1

UPPER_JERK_V  = [  0.5,  0.6, 1.0,  1.5, 2.0,  2.5]
UPPER_JERK_BP = [0.005, 0.05, 0.1, 0.25, 0.5, 0.75]

LOWER_JERK_V =  [   1.0,    1.2,   1.35,    2.0,   2.5,   3.3]
LOWER_JERK_BP = [-0.005, -0.015, -0.025, -0.030, -0.25, -0.75]


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

    # when the last state was off
    if self.long_control_state_last == LongCtrlState.off:
      self.stopping = True
      self.stopping_count = 0
      return

    # 1 second after entering stopping state from non-off states
    if self.stopping_count > 1 / (DT_CTRL * 2):
      self.stopping = True

    self.stopping_count += 1

  def calculate_a_value(self, CC: structs.CarControl) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING or self.CP.radarUnavailable:
      self.desired_accel = CC.actuators.accel
      self.actual_accel = CC.actuators.accel
      return

    if not CC.longActive:
      self.desired_accel = 0.0
      self.actual_accel = 0.0
      self.state.accel_last = 0.0
      return

    # Force zero aReqRaw during StopReq
    if self.stopping:
      self.desired_accel = 0.0
    else:
      self.desired_accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    self.actual_accel = jerk_limited_integrator(self.desired_accel, self.state.accel_last, self.jerk_upper, self.jerk_lower)
    self.state.accel_last = self.actual_accel

    return

  def calculate_jerk(self, CC: structs.CarControl, CS: CarStateBase, long_control_state: LongCtrlState) -> None:
    if not self.CP_SP.flags & HyundaiFlagsSP.LONG_TUNING:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = 5.0
      return

    planned_accel = CC.actuators.accel

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    lower_speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [5.0, 5.0, 2.5]))
    upper_speed_factor = 1.0
    if long_control_state == LongCtrlState.pid:
      upper_speed_factor = float(np.interp(velocity, [0.0, 5.0, 20.0], [1.0, 2.2, 1.0]))

    accel_error = planned_accel - self.state.accel_last

    if planned_accel > 0.005 and accel_error > 0.005:
      upper_jerk = float(np.interp(accel_error, UPPER_JERK_BP, UPPER_JERK_V))
    else:
      upper_jerk = 0.5

    if self.CP.radarUnavailable:
      lower_jerk = 5.0
    elif accel_error < -0.005 or planned_accel < -0.005:
      lower_jerk = float(np.interp(accel_error, LOWER_JERK_BP, LOWER_JERK_V))
    else:
      lower_jerk = 0.5

    desired_jerk_upper = min(upper_jerk, upper_speed_factor)
    desired_jerk_lower = min(lower_jerk, lower_speed_factor)

    self.jerk_upper = ramp_update(self.jerk_upper, desired_jerk_upper)
    self.jerk_lower = desired_jerk_lower
