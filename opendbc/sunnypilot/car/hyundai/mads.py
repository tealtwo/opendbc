from collections import namedtuple

from opendbc.car import structs, DT_CTRL
from opendbc.car.hyundai.carstate import CarState
from opendbc.sunnypilot.car.mads_base import MadsBase
from opendbc.sunnypilot.car.mads_interfaces import MadsControllerBase

MadsDataSP = namedtuple("MadsDataSP",
                        ["enabled_toggle", "lat_active", "disengaging", "paused"])


class Mads(MadsBase):
    def refresh_car_state(self, car_control: structs.CarControl, car_state: CarState):
        """
        This method is called by the CarController to update the car state on the ESCC object.
        The new state is used to update the SCC12 message with the current values of the car state received via ESCC.
        :param car_state:
        :return:
        """
        self.car_state = car_state
        self.car_control = car_control

    def mads_status_update(self, frame) -> MadsDataSP:
      if self.car_control.latActive:
        self.lat_disengage_init = False
      elif self.prev_lat_active:
        self.lat_disengage_init = True

      if not self.lat_disengage_init:
        self.lat_disengage_blink = frame

      paused = self.car_control.madsActive and not self.car_control.latActive
      disengaging = (frame - self.lat_disengage_blink) * DT_CTRL < 1.0 if self.lat_disengage_init else False

      self.prev_lat_active = self.car_control.latActive

      return MadsDataSP(self.car_state.mads_enabled_toggle, self.car_control.latActive, disengaging, paused)

    def update_scc11_message(self, scc11_message):
        """
        Update the scc12 message with the current values of the car state received via ESCC.
        These values come straight from the on-board radar, and are used as a more reliable source for
        AEB / FCA alerts.
        :param scc12_message: the SCC12 message to be sent in dictionary form before being packed
        :return: Nothing. The scc12_message is updated in place.
        """
        scc11_message["AEB_CmdAct"] = self.car_state.escc_cmd_act


class MadsController(MadsControllerBase):
    def init(self):
        self.MADS = Mads(self.CP)

    def update(self, CC: structs.CarControl, CS: CarState, now_nanos):
      self.MADS.refresh_car_state(CC, CS)
