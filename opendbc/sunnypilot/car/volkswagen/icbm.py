from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen import pqcan, mqbcan
from opendbc.car.volkswagen.values import CarControllerParams, VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CS, CP, CC_SP, packer, bus, frame, last_button_frame) -> list[CanData]:
    can_sends = []
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none and self.frame % 10 == 0:
      accel = self.ICBM.sendButton == SendButtonState.increase
      decel = self.ICBM.sendButton == SendButtonState.decrease

      can_sends.append(self.CCS.create_acc_buttons_control(packer, bus, CS.gra_stock_values, cancel=False, resume=False, accel=accel, decel=decel))

    return can_sends
