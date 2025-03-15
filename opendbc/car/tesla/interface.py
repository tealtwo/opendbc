from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.sunnypilot.longitudinal_live_tuner import LongitudinalLiveTuner

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.1

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.experimentalLongitudinalAvailable = True
    if experimental_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value
    ret.liveTuner = LongitudinalLiveTuner(ret, ret.longitudinalTuning.kpV, ret.longitudinalTuning.kiV)

    return ret
