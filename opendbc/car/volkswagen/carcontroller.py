from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_steer_angle_limits_vm, apply_driver_steer_torque_limits
from opendbc.car.common.numpy_fast import clip, interp
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
from opendbc.car.vehicle_model import VehicleModel
import numpy as np
import sys
import os
sunnypilot_path = os.path.join(os.path.dirname(__file__), '..', '..', '..')
sys.path.insert(0, sunnypilot_path)
from openpilot.common.params import Params

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

def get_safety_model():
  # Using NMS Passat Vehicle Model For Angle Safety
  from opendbc.car.volkswagen.interface import CarInterface
  return CarInterface.get_non_essential_params("VOLKSWAGEN_PASSAT_NMS")

def limit_jerk(accel, prev_accel, max_jerk, dt):
  max_delta_accel = max_jerk * dt
  delta_accel = max(-max_delta_accel, min(accel - prev_accel, max_delta_accel))
  return prev_accel + delta_accel

def ECD_Handler(CS, self, ACS_Sta_ADR, ACS_Sollbeschl, vEgo, stopping):
  if (ACS_Sta_ADR == 1 and ACS_Sollbeschl < 0) and \
    ((CS.MOB_Standby and vEgo <= (18 * CV.KPH_TO_MS)) or self.EPB_enable):
      if not self.EPB_enable:  # First frame of EPB entry
          self.EPB_counter = 0
          self.EPB_brake = 0
          self.EPB_enable = 1
          self.EPB_enable_history = [True] * len(self.EPB_enable_history)
          self.EPB_brake_last = ACS_Sollbeschl
      else:
          self.EPB_brake = limit_jerk(-4, self.EPB_brake_last, 0.7, 0.02) if stopping else ACS_Sollbeschl
          self.EPB_brake_last = self.EPB_brake
      self.EPB_counter += 1
  else:
      if self.EPB_enable and self.EPB_counter < 10:  # Keep EPB_enable active for 10 frames
          self.EPB_counter += 1
      else:
          self.EPB_brake = 0
          self.EPB_enable = 0

  if CS.out.gasPressed or CS.out.brakePressed or CS.gra_stock_values["GRA_Abbrechen"]:
    if self.EPB_enable:
      self.ACC_anz_blind = 1
    self.EPB_brake = 0
    self.EPB_enable = 0
    self.EPB_enable_history = [False] * len(self.EPB_enable_history)

  if self.ACC_anz_blind and self.ACC_anz_blind_counter < 150:
    self.ACC_anz_blind_counter += 1
  else:
    self.ACC_anz_blind = 0
    self.ACC_anz_blind_counter = 0

  # Update EPB historical states and calculate EPB_active
  self.EPB_active = int((self.EPB_enable_history[(len(self.EPB_enable_history) - 2)] and not self.EPB_enable) or self.EPB_enable)
  self.EPB_enable_history = self.EPB_enable_history[1:] + [self.EPB_enable]

  return self.EPB_enable, self.EPB_brake, self.EPB_active


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self._params = Params()
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.VM = VehicleModel(get_safety_model())

    self.apply_angle_last = 0
    self.apply_torque_last = 0
    self.gra_acc_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.last_button_frame = 0
    self.accel_last = 0
    self.motor2_frame = 0
    self.gra_acc_counter_last = None
    self.bremse8_counter_last = None
    self.bremse11_counter_last = None
    self.acc_sys_counter_last = None
    self.acc_anz_counter_last = None
    self.ACC_anz_blind = 0
    self.ACC_anz_blind_counter = 0
    self.EPB_brake = 0
    self.EPB_brake_last = 0
    self.EPB_enable = 0
    self.EPB_counter = 0
    self.EPB_enable_history = [False] * 25  # 0.5s history
    self.EPB_active = 0
    self.PLA_status = 0
    self.PLA_entryCounter = 0
    self.PLA_driverExit = False
    self.PLA_driverExit_last = False
    self.CSsteeringAngleDegLast = 0
    self.CSLH3_SignLast = 0
    self.accel_diff = 0
    self.long_deviation = 0
    self.long_jerklimit = 0
    self.HCA_Status = 3
    self.aeb_available = False
    self.awv_warnsymbol = 0
    self.aeb_apply_brake = 0
    self.aeb_enabled = False
    self.aeb_halten = False
    self.aeb_parameter = 0
    self.leadDistanceBars = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []
    hcaLateralControl = self._params.get_bool("pqLatControlToggle")
    eEPBopLongToggle = self._params.get_bool("eEPBopLongToggle")
    eEPBoeLongToggle = self._params.get_bool("eEPBoeLongToggle")
    pqhca5or7Toggle = self._params.get_bool("pqhca5or7Toggle")

    # **** Steering Controls ************************************************ #
    if CS.LH2_Abbr in (2, 7) and CS.out.cruiseState.available:
      self.PLA_driverExit = True
    else:
      self.PLA_driverExit = False
    # HCA (5/7) Lateral Control Logic:
    if self.frame % self.CCP.STEER_STEP == 0 and hcaLateralControl:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero

      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_torque_last == apply_torque:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_torque -= (1, -1)[apply_torque < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_torque) > 0
      else:
        hca_enabled = False
        apply_torque = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0
      if hca_enabled:
        if pqhca5or7Toggle:
          self.HCA_Status = 7
        else:
          self.HCA_Status = 5
      else:
        self.HCA_Status = self.HCA_Status

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_torque_last = apply_torque
      can_sends.append(self.CCS.create_hca_steering_control(self.packer_pt, self.CAN.pt, apply_torque, self.HCA_Status))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # PLA Lateral Control Logic:
    if self.frame % self.CCP.STEER_STEP == 0 and not hcaLateralControl:
      # PLA_status definitions:
      #  10 = reset EPS driver torque override flag
      #  15 = standby
      #  13 = active
      #  11 = activatable, entry request signal. 11 frames required
      if CC.latActive and not self.PLA_driverExit:
        self.PLA_status = 13 if self.PLA_entryCounter >= 11 else 11
        self.PLA_entryCounter += 1 if self.PLA_entryCounter <= 32 else self.PLA_entryCounter
        # retry entry until engagement.
        if CS.LH2_steeringState != 64 and self.PLA_entryCounter >= 30:
          self.PLA_entryCounter = 0
      else:
        self.PLA_status = 10 if self.PLA_driverExit_last and not self.PLA_driverExit else 15  # pulse reset on falling edge
        self.PLA_entryCounter = 0
        self.PLA_driverExit_last = self.PLA_driverExit

      apply_angle = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM) \
        if self.PLA_status == 13 else self.CSsteeringAngleDegLast

      self.apply_angle_last = apply_angle
      self.CSsteeringAngleDegLast = CS.out.steeringAngleDeg
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_angle, self.PLA_status, self.CSLH3_SignLast))
      self.CSLH3_SignLast = CS.LH_3_Sign
    # **** Acceleration Controls ******************************************** #
    # eEPB + opLong (ACC) Control Logic
    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl and eEPBopLongToggle:
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CC.longActive, CC.cruiseControl.override)
      accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      self.accel_diff = (0.0019 * (accel - self.accel_last)) + (1 - 0.0019) * self.accel_diff
      self.long_jerklimit = (0.01 * (clip(abs(accel), 0.7, 2))) + (1 - 0.01) * self.long_jerklimit
      self.long_deviation = clip(CS.out.vEgo / 40, 0, 0.13) * interp(abs(accel - self.accel_diff), [0, .2, 1.], [0.0, 0.0, 0.0])
      if self.CCS == pqcan and CC.longActive and actuators.accel <= 0 and CS.out.vEgoRaw <= 5:
        if not self.EPB_enable:  # first frame of EPB entry
          self.EPB_counter = 0
          self.EPB_brake = 0
          self.EPB_brake_last = accel - (CS.aEgoBremse / 2)
          self.EPB_enable = 1
        else:
          self.EPB_brake = limit_jerk(accel, self.EPB_brake_last, 0.7, 0.02)
          self.EPB_brake_last = self.EPB_brake
      else:
        acc_control = 0 if acc_control != 6 and self.EPB_enable else acc_control  # Pulse ACC status to 0 for one frame
        self.EPB_enable = 0
        self.EPB_brake = 0
      # Increment EPB Counter
      if self.EPB_enable:
        acc_control = 0
        self.EPB_counter = min(self.EPB_counter + 1, 10)
        if self.EPB_counter <= 9:
          acc_control = 0
      else:
        self.EPB_counter = 0
      self.accel_last = accel
      if self.CCS == pqcan:
        can_sends.append(self.CCS.create_epb_control(self.packer_pt, self.CAN.aux, self.EPB_brake, self.EPB_enable))
      can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, accel, acc_control, stopping, starting, CS.esp_hold_confirmation, self.long_deviation, self.long_jerklimit))
    # opLong (ACC) Control Logic
    elif self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl and not eEPBopLongToggle:
      acc_control = self.CCS.stock_acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      self.accel_diff = (0.0019 * (accel - self.accel_last)) + (1 - 0.0019) * self.accel_diff
      self.long_jerklimit = (0.01 * (clip(abs(accel), 0.7, 2))) + (1 - 0.01) * self.long_jerklimit
      self.long_deviation = clip(CS.out.vEgo / 40, 0, 0.13) * interp(abs(accel - self.accel_diff), [0, .2, 1.], [0.0, 0.0, 0.0])
      can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, accel, acc_control, stopping, starting, CS.esp_hold_confirmation, self.long_deviation, self.long_jerklimit))
      # Below here is for OEM+ modification of OEM ACC which allows Follow-to-Stop and SnG using the stock radar via ECD #
    if VolkswagenFlags.PQ and not self.CP.openpilotLongitudinalControl and eEPBoeLongToggle:
      self.stopping = CS.acc_sys_stock["ACS_Anhaltewunsch"] and (CS.out.vEgoRaw <= 2 or self.stopping)
      self.stopped = self.EPB_enable and (CS.out.vEgoRaw == 0 or (self.stopping and self.stopped))
      if CS.acc_sys_stock["COUNTER"] != self.acc_sys_counter_last:
        ECD_Handler(CS, self, CS.acc_sys_stock["ACS_Sta_ADR"], CS.acc_sys_stock["ACS_Sollbeschl"], CS.out.vEgoRaw, self.stopping)
        can_sends.append(self.CCS.filter_ACC_System(self.packer_pt, self.CAN.pt, CS.acc_sys_stock, self.EPB_active))
        can_sends.append(self.CCS.create_epb_control(self.packer_pt, self.CAN.aux, self.EPB_brake, self.EPB_enable))
        can_sends.append(self.CCS.filter_epb1(self.packer_pt, self.CAN.cam, self.stopped))
      if CS.acc_anz_stock["COUNTER"] != self.acc_anz_counter_last:
        can_sends.append(self.CCS.filter_ACC_Anzeige(self.packer_pt, self.CAN.pt, CS.acc_anz_stock, self.ACC_anz_blind))
      if self.frame % 2 or CS.motor2_stock != getattr(self, 'motor2_last', CS.motor2_stock):  # 50hz / 20ms
        can_sends.append(self.CCS.filter_motor2(self.packer_pt, self.CAN.cam, CS.motor2_stock, self.EPB_enable_history[0]))
        if CS.motor2_stock["GRA_Status"] in (1, 2) and self.motor2_last["GRA_Status"] == 0:
          self.EPB_enable_history = [False] * len(
            self.EPB_enable_history)  # disable filter when ECM enters cruise state
      if CS.bremse8_stock["COUNTER"] != self.bremse8_counter_last:
        can_sends.append(self.CCS.filter_bremse8(self.packer_pt, self.CAN.cam, CS.bremse8_stock, self.EPB_enable_history[0]))
      if CS.bremse11_stock["COUNTER"] != self.bremse11_counter_last:
        can_sends.append(self.CCS.filter_bremse11(self.packer_pt, self.CAN.cam, CS.bremse11_stock, self.stopped))
      if CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last:
        can_sends.append(self.CCS.filter_GRA_Neu(self.packer_pt, self.CAN.cam, CS.gra_stock_values, resume=self.stopped and (self.frame % 100 < 50)))
      self.motor2_last = CS.motor2_stock
      self.acc_sys_counter_last = CS.acc_sys_stock["COUNTER"]
      self.acc_anz_counter_last = CS.acc_anz_stock["COUNTER"]
      self.bremse8_counter_last = CS.bremse8_stock["COUNTER"]
      self.bremse11_counter_last = CS.bremse11_stock["COUNTER"]
    # AEB Controller
    self.aeb_available = CS.awv_available
    if self.aeb_available in (0, 14, 10):
      if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        if fcw_alert:
          self.awv_warnsymbol = 1
        else:
          self.awv_warnsymbol = 0
        self.aeb_apply_brake = 0
        self.aeb_enabled = False
        self.aeb_halten = False
        self.aeb_parameter = 0
        can_sends.append(self.CCS.create_aeb_control(self.packer_pt, self.CAN.pt, self.awv_warnsymbol, self.aeb_apply_brake, self.aeb_halten, self.aeb_enabled, self.aeb_parameter))
    # **** HUD Controls ***************************************************** #
    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive, CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      # Handle leadDistanceBars for VW PQ Kombi MFD
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        self.leadDistanceBars = max(3, hud_control.leadDistanceBars) if hud_control.leadDistance != 0 else 0
        if self.leadDistanceBars == 1:
          distanceBars = 2
        elif self.leadDistanceBars == 2:
          distanceBars = 4
        elif self.leadDistanceBars == 3:
          distanceBars = 8
        elif not self.leadDistanceBars or self.leadDistanceBars == 0:
          distanceBars = 0
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CS.out.gasPressed, CC.longActive, CC.cruiseControl.override)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed, lead_distance, distanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values, cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
