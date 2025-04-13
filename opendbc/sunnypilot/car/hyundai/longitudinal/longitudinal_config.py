"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from dataclasses import dataclass

from opendbc.car.hyundai.values import CAR


@dataclass
class CarTuningConfig:
  v_ego_stopping: float = 0.25
  v_ego_starting: float = 0.10
  stopping_decel_rate: float = 0.30
  jerk_limits: tuple[float, float, float] = 0.53, 5.0, 2.0  # (min jerk, max lower jerk, max upper jerk)
  longitudinal_actuator_delay: float = 0.5


# Default configurations for different car types
# Min jerk is set to 0.53 per (Horn et al., 2024)
TUNING_CONFIGS = {
  "CANFD": CarTuningConfig(
    v_ego_starting=0.10,
    stopping_decel_rate=0.35,
  ),
  "EV": CarTuningConfig(
    v_ego_starting=0.10,
    jerk_limits=(0.53, 5.0, 2.2),
  ),
  "HYBRID": CarTuningConfig(
    v_ego_starting=0.12,
    stopping_decel_rate=0.35,
    jerk_limits=(0.53, 5.0, 2.2),
  ),
  "DEFAULT": CarTuningConfig()
}

# Car-specific configs
CAR_SPECIFIC_CONFIGS = {
  CAR.KIA_NIRO_EV: CarTuningConfig(
    v_ego_stopping=0.1,
    stopping_decel_rate=0.05,
    jerk_limits=(0.53, 5.0, 1.6),
    longitudinal_actuator_delay=0.15,
  )
}
