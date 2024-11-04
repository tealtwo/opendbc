from abc import ABC

from opendbc.car import structs
from opendbc.car.sunnypilot.interfaces import CarInterfaceBase, RadarInterfaceBase, CarStateBase, CarControllerBase


class CarInterfaceBase(CarInterfaceBase, ABC):
  def __init__(self, CP: structs.CarParams, CarController, CarState, CPSP: structs.CarParamsSP):
    super().__init__(CP, CarController, CarState)
    self.CPSP = CPSP


class RadarInterfaceBase(RadarInterfaceBase):
  pass


class CarStateBase(CarStateBase, ABC):
  pass


class CarControllerBase(CarControllerBase, ABC):
  pass

