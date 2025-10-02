#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_LENKHILFE_3         0x0D0U   // RX from EPS, for steering angle and driver steering torque
#define MSG_HCA_1               0x0D2U   // TX by OP, Heading Control Assist steering torque
#define MSG_BREMSE_1            0x1A0U   // RX from ABS, for ego speed
#define MSG_MOTOR_2             0x288U   // RX from ECU, for CC state and brake switch state
#define MSG_ACC_SYSTEM          0x368U   // TX by OP, longitudinal acceleration controls
#define MSG_MOTOR_3             0x380U   // RX from ECU, for driver throttle input
#define MSG_GRA_NEU             0x38AU   // TX by OP, ACC control buttons for cancel/resume
#define MSG_MOTOR_5             0x480U   // RX from ECU, for ACC main switch state
#define MSG_ACC_GRA_ANZEIGE     0x56AU   // TX by OP, ACC HUD
#define MSG_LDW_1               0x5BEU   // TX by OP, Lane line recognition and text alerts
#define MSG_EPB_1               0x5C0U   // TX by OP, EPB/ECD control.
#define MSG_BREMSE_8            0x1ACU   // TX by OP, spoofing radar
#define MSG_BREMSE_11           0x5B7U   // TX by OP, spoofing radar

static uint32_t volkswagen_pq_get_checksum(const CANPacket_t *msg) {
  return (uint32_t)msg->data[(msg->addr == MSG_MOTOR_5) ? 7 : 0];
}

static uint8_t volkswagen_pq_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;

  if (msg->addr == MSG_LENKHILFE_3) {
    counter = (uint8_t)(msg->data[1] & 0xF0U) >> 4;
  } else if (msg->addr == MSG_GRA_NEU) {
    counter = (uint8_t)(msg->data[2] & 0xF0U) >> 4;
  } else {
  }

  return counter;
}

static uint32_t volkswagen_pq_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = 0U;
  int checksum_byte = (msg->addr == MSG_MOTOR_5) ? 7 : 0;

  // Simple XOR over the payload, except for the byte where the checksum lives.
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      checksum ^= (uint8_t)msg->data[i];
    }
  }

  return checksum;
}

static safety_config volkswagen_pq_init(uint16_t param) {
  // Transmit of GRA_Neu is allowed on bus 0 and 2 to keep compatibility with gateway and camera integration
  static const CanMsg VOLKSWAGEN_PQ_STOCK_TX_MSGS[] = {{MSG_HCA_1, 0, 5, .check_relay = true}, {MSG_LDW_1, 0, 8, .check_relay = true},
                                                 {MSG_GRA_NEU, 0, 4, .check_relay = false}, {MSG_GRA_NEU, 2, 4, .check_relay = false},
                                                 {MSG_ACC_GRA_ANZEIGE, 0, 8, .check_relay = true}, {MSG_ACC_SYSTEM, 0, 8, .check_relay = true},
                                                 {MSG_MOTOR_2, 2, 8, .check_relay = true}, {MSG_EPB_1, 1, 8, .check_relay = true},
                                                 {MSG_EPB_1, 2, 8, .check_relay = true}, {MSG_BREMSE_8, 2, 8, .check_relay = true},
                                                 {MSG_BREMSE_11, 2, 8, .check_relay = true}};

  static const CanMsg VOLKSWAGEN_PQ_LONG_TX_MSGS[] =  {{MSG_HCA_1, 0, 5, .check_relay = true}, {MSG_LDW_1, 0, 8, .check_relay = true},
                                                {MSG_ACC_SYSTEM, 0, 8, .check_relay = true}, {MSG_ACC_GRA_ANZEIGE, 0, 8, .check_relay = true},
                                                {MSG_MOTOR_2, 2, 8, .check_relay = true}, {MSG_EPB_1, 1, 8, .check_relay = true},
                                                {MSG_EPB_1, 2, 8, .check_relay = true}, {MSG_BREMSE_8, 2, 8, .check_relay = true},
                                                {MSG_BREMSE_11, 2, 8, .check_relay = true}};

  static RxCheck volkswagen_pq_rx_checks[] = {
    {.msg = {{MSG_LENKHILFE_3, 0, 6, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_BREMSE_1, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_3, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_5, 0, 8, 50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_NEU, 0, 4, 30U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  UNUSED(param);

  volkswagen_set_button_prev = false;
  volkswagen_resume_button_prev = false;

#ifdef ALLOW_DEBUG
  volkswagen_longitudinal = GET_FLAG(param, FLAG_VOLKSWAGEN_LONG_CONTROL);
#endif
  return volkswagen_longitudinal ? BUILD_SAFETY_CFG(volkswagen_pq_rx_checks, VOLKSWAGEN_PQ_LONG_TX_MSGS) : \
                                   BUILD_SAFETY_CFG(volkswagen_pq_rx_checks, VOLKSWAGEN_PQ_STOCK_TX_MSGS);
}

static void volkswagen_pq_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state from speed value.
    // Signal: Bremse_1.Geschwindigkeit_neu__Bremse_1_
    if (msg->addr == MSG_BREMSE_1) {
      int speed = ((msg->data[2] & 0xFEU) >> 1) | (msg->data[3] << 7);
      vehicle_moving = speed > 0;
    }

    // Update driver input torque samples
    // Signal: Lenkhilfe_3.LH3_LM (absolute torque)
    // Signal: Lenkhilfe_3.LH3_LMSign (direction)
    if (msg->addr == MSG_LENKHILFE_3) {
      int torque_driver_new = msg->data[2] | ((msg->data[3] & 0x3U) << 8);
      int sign = (msg->data[3] & 0x4U) >> 2;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (volkswagen_longitudinal) {
      if (msg->addr == MSG_MOTOR_5) {
        // ACC main switch on is a prerequisite to enter controls, exit controls immediately on main switch off
        // Signal: Motor_5.GRA_Hauptschalter
        acc_main_on = GET_BIT(msg, 50U);
        if (!acc_main_on) {
          controls_allowed = false;
        }
      }

      if (msg->addr == MSG_GRA_NEU) {
        // If ACC main switch is on, enter controls on falling edge of Set or Resume
        // Signal: GRA_Neu.GRA_Neu_Setzen
        // Signal: GRA_Neu.GRA_Neu_Recall
        bool set_button = GET_BIT(msg, 16U);
        bool resume_button = GET_BIT(msg, 17U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
        // Exit controls on rising edge of Cancel, override Set/Resume if present simultaneously
        // Signal: GRA_ACC_01.GRA_Abbrechen
        if (GET_BIT(msg, 9U)) {
          controls_allowed = false;
        }
      }
    } else {
      if (msg->addr == MSG_MOTOR_2) {
        // Enter controls on rising edge of stock ACC, exit controls if stock ACC disengages
        // Signal: Motor_2.GRA_Status
        int acc_status = (msg->data[2] & 0xC0U) >> 6;
        bool cruise_engaged = (acc_status == 1) || (acc_status == 2);
        pcm_cruise_check(cruise_engaged);
      }
    }

    // Signal: Motor_3.Fahrpedal_Rohsignal
    if (msg->addr == MSG_MOTOR_3) {
      gas_pressed = (msg->data[2]);
    }

    // Signal: Motor_2.Bremslichtschalter
    if (msg->addr == MSG_MOTOR_2) {
      brake_pressed = (msg->data[2] & 0x1U);
    }
  }
}

static bool volkswagen_pq_tx_hook(const CANPacket_t *msg) {
  // Lateral Torque Limits (HCA)
  const TorqueSteeringLimits VWTorqueLimits = {
    .max_torque = 300,               // 3.0 Nm (EPS side max of 3.0Nm with fault if violated)
    .max_rt_delta = 113,             // 6 max rate up * 50Hz send rate * 250000 RT interval / 1000000 = 75 ; 125 * 1.5 for safety pad = 113
    .max_rate_up = 6,                // 3.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .max_rate_down = 10,             // 5.0 Nm/s RoC limit (EPS rack has own soft-limit of 5.0 Nm/s)
    .driver_torque_multiplier = 3,
    .driver_torque_allowance = 80, // Max Driver Torque Input Before OP Gives Up
    .type = TorqueDriverLimited,
  };
  // Lateral Angle Limits (PLA) 540 deg total, 10deg/frame max rotation
  const AngleSteeringLimits VWAngleLimits = {
    .max_angle = 5400,  // 540 deg
    .angle_deg_to_can = 10, // 10deg/f
    .frequency = 50U, // 50hz TX rate
  };
  // Based off Volkswagen Passat NMS to match openpilot
  const AngleSteeringParams VWSteeringParams = {
    .slip_factor = -0.000580374383851451,  // calc_slip_factor(VM) | FIXME: Stolen From Tesla (Actual Val: -0.00143398)
    .steer_ratio = 18.4, // Volkswagen Passat NMS SteerRatio
    .wheelbase = 2.80, // Volkswagen Passat NMS WheelBase
  };
  // Longitudinal Limits
  // Acceleration in m/s^2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_PQ_LONG_LIMITS = {
    .max_accel = 2000, // 2 m/s^2 Max Acceleration
    .min_accel = -3500, // 3.5 m/s^2 Max Deceleration
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  bool tx = true; // Always TX if not Checked
  // Safety check for Lateral Control, Torque|Angle
  // Signal: HCA_1.LW_OffSet (requested torque)
  // Signal: HCA_1.LM_OffSet (requested angle)
  // Signal: HCA_1.LM_OffSign (direction)
  if (msg->addr == MSG_HCA_1) {
    uint32_t hca_status = ((msg->data[1] >> 4) & 0xFU); // HCA Status
    // Check for Angle Control
    bool angle_steering = (hca_status == 10U || hca_status == 11U || hca_status == 13U || hca_status == 15U); // Check for HCA->PLA Status
    bool angle_steering_active = (hca_status == 11U || hca_status == 13U) // Check for Active HCA->PLA Status
    // LW_OffSet (Torque, HCA) Safety Check:
    if (!angle_steering) {
      int desired_torque = msg->data[2] | ((msg->data[3] & 0x7FU) << 8); // HC1_LW_OffSet (Req Torque)
      desired_torque = desired_torque / 32; // Scaling factor for PQ to centi-Nm (/32)
      int sign = (msg->data[3] & 0x80U) >> 7; // Sign Bit
      if (sign == 1) {
        desired_torque *= -1; // Apply Sign to Desired Torque
      }
      // Check for Torque Control
      bool steer_req = ((hca_status == 5U) || (hca_status == 7U)); //
      if ((steer_torque_cmd_checks(desired_torque, steer_req, VWTorqueLimits))) {
        tx = false; // Block Requested Torque if Safety Check Failed
      }
    // LM_OffSet (Angle, PLA) Safety Check:
    } else {
      int desired_angle = msg->data[2] | ((msg->data[3] & 0x7FU) << 8); // HC1_LM_OffSet (Req Angle)
      desired_angle = desired_angle * 0.04375 // Scaling factor for PQ to Angle (* 0.04375)
      int sign = (msg->data[3] & 0x80U) >> 7; // Sign Bit
      if (sign == 1) {
        desired_angle *= -1; // Apply Sign to Desired Angle
      }
      if (steer_angle_cmd_checks_vm(desired_angle, angle_steering_active, VWAngleLimits, VWSteeringParams)) {
        tx = false; // Block Requested Angle if Safety Check Failed
      }
    }
  }
  // Safety check for acceleration commands
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s2 boundaries
  if ((msg->addr == MSG_ACC_SYSTEM) && volkswagen_longitudinal) {
    // Signal: ACC_System.ACS_Sollbeschl (acceleration in m/s2, scale 0.005, offset -7.22)
    int desired_accel = ((((msg->data[4] & 0x7U) << 8) | msg->data[3]) * 5U) - 7220U;

    if (longitudinal_accel_checks(desired_accel, VOLKSWAGEN_PQ_LONG_LIMITS)) {
      tx = false;
    }
  }
  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((msg->addr == MSG_GRA_NEU) && !controls_allowed) {
    // Signal: GRA_Neu.GRA_Neu_Setzen
    // Signal: GRA_Neu.GRA_Neu_Recall
    if (GET_BIT(msg, 16U) || GET_BIT(msg, 17U)) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks volkswagen_pq_hooks = {
  .init = volkswagen_pq_init,
  .rx = volkswagen_pq_rx_hook,
  .tx = volkswagen_pq_tx_hook,
  .get_counter = volkswagen_pq_get_counter,
  .get_checksum = volkswagen_pq_get_checksum,
  .compute_checksum = volkswagen_pq_compute_checksum,
};
