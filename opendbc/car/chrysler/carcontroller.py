from opendbc.car.common.conversions import Conversions as CV

from opendbc.can.packer import CANPacker
from opendbc.car import DT_CTRL, apply_meas_steer_torque_limits
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import RAM_CARS, CarControllerParams, STEER_TO_ZERO
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.apply_steer_last = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_name)
    self.params = CarControllerParams(CP)

    self.spoof_speed = 0
    self.spoof_speed_increment = 0.2
    self.spoof_speed_threshold = 7

  def update(self, CC, CS, now_nanos):
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if (self.frame - self.last_button_frame)*DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation
      if CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.extend(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      speed_logic = self.spoof_speed if self.CP.carFingerprint in STEER_TO_ZERO else CS.out.vEgo
      if CS.out.vEgo < self.CP.minSteerSpeed or \
            (self.CP.carFingerprint in STEER_TO_ZERO and self.spoof_speed < self.CP.minEnableSpeed):
        lkas_control_bit = False
      elif (speed_logic >= self.CP.minEnableSpeed) or (self.CP.carFingerprint in STEER_TO_ZERO):
        lkas_control_bit = CC.latActive

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_steer = int(round(CC.actuators.steer * self.params.STEER_MAX))
      apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit or not self.lkas_control_bit_prev:
        apply_steer = 0
      self.apply_steer_last = apply_steer

      can_sends.extend(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_steer), lkas_control_bit, int(self.frame/self.params.STEER_STEP)))

    if self.CP.carFingerprint in STEER_TO_ZERO and self.frame % 2 == 0:
      if lkas_active and CS.out.vEgo > self.CP.minSteerSpeed:
        if self.spoof_speed < self.spoof_speed_threshold:
          self.spoof_speed = max(self.spoof_speed, CS.out.vEgo) + self.spoof_speed_increment
        else:
          self.spoof_speed = self.CP.minEnableSpeed
      else:
        self.spoof_speed = CS.out.vEgo
      can_sends.append(chryslercan.create_speed_spoof(self.packer, self.spoof_speed * CV.MS_TO_KPH))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.params.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    return new_actuators, can_sends
