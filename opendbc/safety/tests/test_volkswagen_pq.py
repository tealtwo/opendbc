#!/usr/bin/env python3
import unittest

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags, CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda, away_round

MSG_LENKHILFE_3 = 0x0D0       # RX from EPS, for steering angle and driver steering torque
MSG_HCA_1 = 0x0D2             # TX by OP, Heading Control Assist steering torque
MSG_BREMSE_1 = 0x1A0          # RX from ABS, for ego speed
MSG_MOTOR_2 = 0x288           # RX from ECU, for CC state and brake switch state
MSG_ACC_SYSTEM = 0x368        # TX by OP, longitudinal acceleration controls
MSG_MOTOR_3 = 0x380           # RX from ECU, for driver throttle input
MSG_GRA_NEU = 0x38A           # TX by OP, ACC control buttons for cancel/resume
MSG_MOTOR_5 = 0x480           # RX from ECU, for ACC main switch state
MSG_ACC_GRA_ANZEIGE = 0x56A   # TX by OP, ACC HUD
MSG_LDW_1 = 0x5BE             # TX by OP, Lane line recognition and text alerts
MSG_EPB_1 = 0x5C0             # TX by OP, eEPB for FtS & SnG on MK60EC1-CC using ECD
MSG_BREMSE_8 = 0x1AC          # TX by OP, spoof radar, RX by OP, ECD feedback
MSG_BREMSE_11 = 0x5B7         # TX by OP, spoof radar

class TestVolkswagenPqSafetyBase(common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  cruise_engaged = False

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1)}
  # Torque (HCA) Control Limits
  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 113

  # Angle (PLA) Control Limits
  # STEER_ANGLE_MAX = 495  # deg
  # DEG_TO_CAN = 10
  # Volkswagen uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  # ANGLE_RATE_BP = None
  # ANGLE_RATE_UP = None
  # ANGLE_RATE_DOWN = None

  # Real Time Limits
  # LATERAL_FREQUENCY = 50  # Hz

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  # def _get_steer_cmd_angle_max(self, speed):
  #  return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  # def setUp(self):
  #  self.VM = VehicleModel(get_safety_CP())
  #  self.packer = CANPackerPanda("vw_pq")
  #  self.define = CANDefine("vw_pq")

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  # Ego speed (Bremse_1)
  def _speed_msg(self, speed):
    values = {"BR1_Rad_kmh": speed}
    return self.packer.make_can_msg_panda("Bremse_1", 0, values)

  # Brake light switch (shared message Motor_2)
  def _user_brake_msg(self, brake):
    # since this signal is used for engagement status, preserve current state
    return self._motor_2_msg(brake_pressed=brake, cruise_engaged=self.safety.get_controls_allowed())

  # ACC engaged status (shared message Motor_2)
  def _pcm_status_msg(self, enable):
    self.__class__.cruise_engaged = enable
    return self._motor_2_msg(cruise_engaged=enable)

  # Acceleration request to drivetrain coordinator
  def _accel_msg(self, accel):
    values = {"ACS_Sollbeschl": accel}
    return self.packer.make_can_msg_panda("ACC_System", 0, values)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"LH3_LM": abs(torque), "LH3_LMSign": torque < 0}
    return self.packer.make_can_msg_panda("Lenkhilfe_3", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1, hca_status=5):
    values = {"LW_Offset": abs(torque), "LM_OffSign": torque < 0, "HCA_Status": hca_status if steer_req else 3}
    return self.packer.make_can_msg_panda("HCA_1", 0, values)

  # openpilot steering output angle
  # def _angle_cmd_msg(self, angle, steer_req=1, hca_status=13):
  #  values = {"LM_Offset": abs(angle), "LM_OffSign": angle < 0, "HCA_Status": hca_status if steer_req else 15}
  #  return self.packer.make_can_msg_panda("HCA_1", 0, values)

  # ACC engagement and brake light switch status
  # Called indirectly for compatibility with common.py tests
  def _motor_2_msg(self, brake_pressed=False, cruise_engaged=False):
    values = {"MO2_BLS": brake_pressed, "MO2_Sta_GRA": cruise_engaged}
    return self.packer.make_can_msg_panda("Motor_2", 0, values)

  # ACC main switch status
  def _motor_5_msg(self, main_switch=False):
    values = {"GRA_Hauptschalter": main_switch}
    return self.packer.make_can_msg_panda("Motor_5", 0, values)

  # Driver throttle input (Motor_3)
  def _user_gas_msg(self, gas):
    values = {"Fahrpedal_Rohsignal": gas}
    return self.packer.make_can_msg_panda("Motor_3", 0, values)

  # Cruise control buttons (GRA_Neu)
  def _button_msg(self, _set=False, resume=False, cancel=False, bus=2):
    values = {"GRA_Neu_Setzen": _set, "GRA_Recall": resume, "GRA_Abbrechen": cancel}
    return self.packer.make_can_msg_panda("GRA_Neu", bus, values)

  def test_torque_measurements(self):
    # TODO: make this test work with all cars
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())


class TestVolkswagenPqStockSafety(TestVolkswagenPqSafetyBase):
  # Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration, OEM+ allows EPB/ECD for FtS & SnG on CC MK60EC1
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_GRA_NEU, 0], [MSG_GRA_NEU, 2], [MSG_LDW_1, 0], [MSG_ACC_GRA_ANZEIGE, 0], [MSG_ACC_SYSTEM, 0], [MSG_MOTOR_2, 2], [MSG_EPB_1, 1], [MSG_EPB_1, 2], [MSG_BREMSE_8, 2], [MSG_BREMSE_11, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_BREMSE_8, MSG_MOTOR_2, MSG_BREMSE_11, MSG_EPB_1], 2: [MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE, MSG_BREMSE_11, MSG_MOTOR_2, MSG_EPB_1]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE), 1: (MSG_EPB_1,), 2: (MSG_MOTOR_2, MSG_EPB_1, MSG_BREMSE_8, MSG_BREMSE_11)}

  def setUp(self):
    self.packer = CANPackerPanda("vw_pq")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))
    self.assertFalse(self._tx(self._button_msg(_set=True)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestVolkswagenPqLongSafety(TestVolkswagenPqSafetyBase, common.LongitudinalAccelSafetyTest):
  TX_MSGS = [[MSG_HCA_1, 0], [MSG_LDW_1, 0], [MSG_ACC_SYSTEM, 0], [MSG_ACC_GRA_ANZEIGE, 0], [MSG_MOTOR_2, 2], [MSG_EPB_1, 1], [MSG_EPB_1, 2], [MSG_BREMSE_8, 2], [MSG_BREMSE_11, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_BREMSE_8, MSG_MOTOR_2, MSG_BREMSE_11, MSG_EPB_1], 2: [MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE, MSG_BREMSE_11, MSG_MOTOR_2, MSG_EPB_1]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_1, MSG_LDW_1, MSG_ACC_SYSTEM, MSG_ACC_GRA_ANZEIGE), 1: (MSG_EPB_1,), 2: (MSG_MOTOR_2, MSG_EPB_1, MSG_BREMSE_8, MSG_BREMSE_11)}
  INACTIVE_ACCEL = 3.01

  def setUp(self):
    self.packer = CANPackerPanda("vw_pq")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._motor_5_msg(main_switch=False))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self._rx(self._button_msg(bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._motor_5_msg(main_switch=True))
      self._rx(self._button_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._button_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._motor_5_msg(main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._motor_5_msg(main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_torque_cmd_enable_variants(self):
    # The EPS rack accepts either 5 or 7 for an enabled status, with different low speed tuning behavior
    self.safety.set_controls_allowed(1)
    for enabled_status in (5, 7):
      self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP, steer_req=1, hca_status=enabled_status)),
                      f"torque cmd rejected with {enabled_status=}")


if __name__ == "__main__":
  unittest.main()
