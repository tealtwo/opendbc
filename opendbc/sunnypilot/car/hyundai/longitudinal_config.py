from abc import ABC
from dataclasses import dataclass

from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.interfaces import CarControllerBase


@dataclass
class CarTuningConfig:
  vego_stopping: float
  vego_starting: float
  stopping_decel_rate: float
  start_accel: float
  jerk_limits: tuple[float, float, float]  # (min, max lower jerk, max upper jerk)
  brake_response: tuple[float, float, float, float]
  accel_limits: tuple[float, float]  # (min, max)

# Default configurations for different car types
# Min jerk is set to 0.60 per (Horn et al., 2024)
TUNING_CONFIGS = {
  "EV": CarTuningConfig(
    vego_stopping=0.25,
    vego_starting=0.1,
    stopping_decel_rate=0.20,
    start_accel=1.6,
    jerk_limits=(0.60, 5.0, 3.0),
    brake_response=(1.2, 1.8, 2.5, 3.5),
    accel_limits=(-3.5, 2.0),
  ),
  "HYBRID": CarTuningConfig(
    vego_stopping=0.25,
    vego_starting=0.12,
    stopping_decel_rate=0.20,
    start_accel=1.5,
    jerk_limits=(0.60, 5.0, 3.0),
    brake_response=(1.25, 1.85, 2.55, 3.5),
    accel_limits=(-3.5, 2.0),
  ),
  "ICE": CarTuningConfig(
    vego_stopping=0.25,
    vego_starting=0.1,
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
    vego_stopping=0.25,
    vego_starting=0.10,
    stopping_decel_rate=0.05,
    start_accel=1.0,
    jerk_limits=(0.5, 5.0, 3.0),
    brake_response=(1.3, 1.5, 2.5, 3.5),
    accel_limits=(-3.5, 2.0),
  )
}


class Cartuning(CarControllerBase, ABC):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)

  @staticmethod
  def get_car_config(CP: structs.CarParams) -> CarTuningConfig:
    # Get car type flags from specific configs or determine from car flags
    car_config = CAR_SPECIFIC_CONFIGS.get(CP.carFingerprint)
    # If car is not in specific configs, determine from flags
    if car_config is None:
      if CP.flags & HyundaiFlags.EV:
        car_config = TUNING_CONFIGS["EV"]
      elif CP.flags & HyundaiFlags.HYBRID:
        car_config = TUNING_CONFIGS["HYBRID"]
      else:
        car_config = TUNING_CONFIGS["ICE"]

    return car_config
