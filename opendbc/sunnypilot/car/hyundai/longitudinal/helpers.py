from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.longitudinal_config import CarTuningConfig, TUNING_CONFIGS, CAR_SPECIFIC_CONFIGS

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
