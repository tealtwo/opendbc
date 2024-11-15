from abc import ABC, abstractmethod

from opendbc.car.interfaces import CarStateBase
from opendbc.car import structs, ButtonType
from opendbc.sunnypilot.mads.helpers import MadsParams


class MadsStateBase(ABC):
  def __init__(self):
    self.alt_button = 0
    self.mads_enabled_toggle = MadsParams().read_param("Mads")
    self.main_cruise_enabled = False

  def get_main_cruise(self, ret: structs.CarState) -> bool:
    if any(be.type == ButtonType.mainCruise and be.pressed for be in ret.buttonEvents):
      self.main_cruise_enabled = not self.main_cruise_enabled

    return self.main_cruise_enabled if ret.cruiseState.available else False


class MadsControllerBase(ABC):
  def __init__(self, CP):
    self.CP = CP
    self.MADS = None
    self.init()
    if self.MADS is None:
      raise ValueError("MADS is not initialized")

  @abstractmethod
  def init(self):
    raise NotImplementedError

  @abstractmethod
  def update(self, CC: structs.CarControl, CS: CarStateBase, now_nanos: int) -> tuple[structs.CarControl.Actuators, list[CanData]]:
    raise NotImplementedError
