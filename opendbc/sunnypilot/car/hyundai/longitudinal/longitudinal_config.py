"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  v_ego_stopping: float
  v_ego_starting: float
  stopping_decel_rate: float
  start_accel: float
  jerk_limits: tuple[float, float, float]  # (min, max lower jerk, max upper jerk)
  brake_response: tuple[float, float, float, float]
  accel_limits: tuple[float, float]  # (min, max)


# Default configurations for different car types
# Min jerk is set to 0.60 per (Horn et al., 2024)
TUNING_CONFIGS = {
  "EV": CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.1,
    stopping_decel_rate=0.20,
    start_accel=1.6,
    jerk_limits=(0.60, 5.0, 3.0),
    brake_response=(1.2, 1.8, 2.5, 3.5),
    accel_limits=(-3.5, 2.0),
  ),
  "HYBRID": CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.12,
    stopping_decel_rate=0.20,
    start_accel=1.5,
    jerk_limits=(0.60, 5.0, 3.0),
    brake_response=(1.25, 1.85, 2.55, 3.5),
    accel_limits=(-3.5, 2.0),
  ),
  "ICE": CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.1,
    stopping_decel_rate=0.30,
    start_accel=1.6,
    jerk_limits=(0.60, 5.0, 3.0),
    brake_response=(1.3, 1.9, 2.65, 3.5),
    accel_limits=(-3.5, 2.0),
  )
}

# Car-specific configs
CAR_SPECIFIC_CONFIGS = {
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.25,
    v_ego_starting=0.10,
    stopping_decel_rate=0.05,
    start_accel=1.0,
    jerk_limits=(0.5, 5.0, 3.0),
    brake_response=(1.3, 1.5, 2.5, 3.5),
    accel_limits=(-3.5, 2.0),
  )
}
