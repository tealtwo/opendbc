from opendbc.car import structs
from opendbc.car.sunnypilot.interfaces import CarStateBase


class CarState(CarStateBase):
  def update(self, *_) -> structs.CarState:
    return structs.CarState()
