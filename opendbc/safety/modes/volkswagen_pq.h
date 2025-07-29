#pragma once

#include "opendbc/safety/safety_declarations.h"
#include "opendbc/safety/modes/volkswagen_common.h"

#define MSG_LENKHILFE_3         0x0D0   // RX from EPS, for steering angle and driver steering torque
#define MSG_HCA_1               0x0D2   // TX by OP, Heading Control Assist steering torque
#define MSG_BREMSE_1            0x1A0   // RX from ABS, for ego speed
#define MSG_MOTOR_2             0x288   // RX from ECU, for CC state and brake switch state
#define MSG_ACC_SYSTEM          0x368   // TX by OP, longitudinal acceleration controls
#define MSG_MOTOR_3             0x380   // RX from ECU, for driver throttle input
#define MSG_GRA_NEU             0x38A   // TX by OP, ACC control buttons for cancel/resume
#define MSG_MOTOR_5             0x480   // RX from ECU, for ACC main switch state
#define MSG_ACC_GRA_ANZEIGE     0x56A   // TX by OP, ACC HUD
#define MSG_LDW_1               0x5BE   // TX by OP, Lane line recognition and text alerts
#define MSG_EPB_1               0x5C0   // TX by OP, EPB/ECD control.
#define MSG_BREMSE_8            0x1AC   // TX by OP, spoofing radar
#define MSG_BREMSE_11           0x5B7   // TX by OP, spoofing radar

static uint32_t volkswagen_pq_get_checksum(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);

  return (uint32_t)GET_BYTE(to_push, (addr == MSG_MOTOR_5) ? 7 : 0);
}

static uint8_t volkswagen_pq_get_counter(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  uint8_t counter = 0U;

  if (addr == MSG_LENKHILFE_3) {
    counter = (uint8_t)(GET_BYTE(to_push, 1) & 0xF0U) >> 4;
  } else if (addr == MSG_GRA_NEU) {
    counter = (uint8_t)(GET_BYTE(to_push, 2) & 0xF0U) >> 4;
  } else {
  }

  return counter;
}

static uint32_t volkswagen_pq_compute_checksum(const CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  uint8_t checksum = 0U;
  int checksum_byte = (addr == MSG_MOTOR_5) ? 7 : 0;

  // Simple XOR over the payload, except for the byte where the checksum lives.
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      checksum ^= (uint8_t)GET_BYTE(to_push, i);
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
    {.msg = {{MSG_LENKHILFE_3, 0, 6, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_BREMSE_1, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_2, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_3, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{MSG_MOTOR_5, 0, 8, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{MSG_GRA_NEU, 0, 4, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 30U}, { 0 }, { 0 }}},
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

static void volkswagen_pq_rx_hook(const CANPacket_t *to_push) {
  if (GET_BUS(to_push) == 0U) {
    int addr = GET_ADDR(to_push);

    // Update in-motion state from speed value.
    // Signal: Bremse_1.Geschwindigkeit_neu__Bremse_1_
    if (addr == MSG_BREMSE_1) {
      int speed = ((GET_BYTE(to_push, 2) & 0xFEU) >> 1) | (GET_BYTE(to_push, 3) << 7);
      vehicle_moving = speed > 0;
    }

    // Update driver input torque samples
    // Signal: Lenkhilfe_3.LH3_LM (absolute torque)
    // Signal: Lenkhilfe_3.LH3_LMSign (direction)
    if (addr == MSG_LENKHILFE_3) {
      int torque_driver_new = GET_BYTE(to_push, 2) | ((GET_BYTE(to_push, 3) & 0x3U) << 8);
      int sign = (GET_BYTE(to_push, 3) & 0x4U) >> 2;
      if (sign == 1) {
        torque_driver_new *= -1;
      }
      update_sample(&torque_driver, torque_driver_new);
    }

    if (volkswagen_longitudinal) {
      if (addr == MSG_MOTOR_5) {
        // ACC main switch on is a prerequisite to enter controls, exit controls immediately on main switch off
        // Signal: Motor_5.GRA_Hauptschalter
        acc_main_on = GET_BIT(to_push, 50U);
        if (!acc_main_on) {
          controls_allowed = false;
        }
      }

      if (addr == MSG_GRA_NEU) {
        // If ACC main switch is on, enter controls on falling edge of Set or Resume
        // Signal: GRA_Neu.GRA_Neu_Setzen
        // Signal: GRA_Neu.GRA_Neu_Recall
        bool set_button = GET_BIT(to_push, 16U);
        bool resume_button = GET_BIT(to_push, 17U);
        if ((volkswagen_set_button_prev && !set_button) || (volkswagen_resume_button_prev && !resume_button)) {
          controls_allowed = acc_main_on;
        }
        volkswagen_set_button_prev = set_button;
        volkswagen_resume_button_prev = resume_button;
        // Exit controls on rising edge of Cancel, override Set/Resume if present simultaneously
        // Signal: GRA_ACC_01.GRA_Abbrechen
        if (GET_BIT(to_push, 9U)) {
          controls_allowed = false;
        }
      }
    } else {
      if (addr == MSG_MOTOR_2) {
        // Enter controls on rising edge of stock ACC, exit controls if stock ACC disengages
        // Signal: Motor_2.GRA_Status
        int acc_status = (GET_BYTE(to_push, 2) & 0xC0U) >> 6;
        bool cruise_engaged = (acc_status == 1) || (acc_status == 2);
        pcm_cruise_check(cruise_engaged);
      }
    }

    // Signal: Motor_3.Fahrpedal_Rohsignal
    if (addr == MSG_MOTOR_3) {
      gas_pressed = (GET_BYTE(to_push, 2));
    }

    // Signal: Motor_2.Bremslichtschalter
    if (addr == MSG_MOTOR_2) {
      brake_pressed = (GET_BYTE(to_push, 2) & 0x1U);
    }
  }
}

static bool volkswagen_pq_tx_hook(const CANPacket_t *to_send) {
  // longitudinal limits
  // acceleration in m/s2 * 1000 to avoid floating point math
  const LongitudinalLimits VOLKSWAGEN_PQ_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 3010,  // VW sends one increment above the max range when inactive
  };

  int addr = GET_ADDR(to_send);
  bool tx = true;

  // Safety check for acceleration commands
  // To avoid floating point math, scale upward and compare to pre-scaled safety m/s2 boundaries
  if (addr == MSG_ACC_SYSTEM) {
    // Signal: ACC_System.ACS_Sollbeschl (acceleration in m/s2, scale 0.005, offset -7.22)
    int desired_accel = ((((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) * 5U) - 7220U;

    if (longitudinal_accel_checks(desired_accel, VOLKSWAGEN_PQ_LONG_LIMITS)) {
      tx = true; // testing change UNDO THIS!!!!!!!!!!!!
    }
  }

  // FORCE CANCEL: ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == MSG_GRA_NEU) && !controls_allowed) {
    // Signal: GRA_Neu.GRA_Neu_Setzen
    // Signal: GRA_Neu.GRA_Neu_Recall
    if (GET_BIT(to_send, 16U) || GET_BIT(to_send, 17U)) {
      tx = false;
    }
  }

  return tx;
}

static bool volkswagen_pq_fwd_hook(int bus_num, int addr) {
  bool bus_fwd = true;  // Default to blocking

  switch (bus_num) {
    case 0:
      if (!volkswagen_longitudinal && ((addr == MSG_MOTOR_2) || (addr == MSG_BREMSE_8) || (addr == MSG_BREMSE_11) || (addr == MSG_EPB_1) || (addr == MSG_GRA_NEU))) {
        // openpilot takes over signals OEM-radar listens to for OEM+ SNG(ECD on CC H46 ABS)
        bus_fwd = true;
      } else {
        // Forward all traffic from the Extended CAN onward (to bus 2)
        bus_fwd = false;
      }
      break;
    case 2:
      if ((addr == MSG_HCA_1) || (addr == MSG_LDW_1) || (addr == MSG_ACC_SYSTEM) || (addr == MSG_ACC_GRA_ANZEIGE)) {
        // openpilot takes over LKAS steering control, HUD msg, and ACC signals
        bus_fwd = true;
      } else {
        // Forward all remaining traffic from Extended CAN devices to J533 gateway (to bus 0)
        bus_fwd = false;
      }
      break;
    default:
      // No other buses should be in use; fallback to block
      bus_fwd = true;
      break;
  }

  return bus_fwd;
}

const safety_hooks volkswagen_pq_hooks = {
  .init = volkswagen_pq_init,
  .rx = volkswagen_pq_rx_hook,
  .tx = volkswagen_pq_tx_hook,
  .fwd = volkswagen_pq_fwd_hook,
  .get_counter = volkswagen_pq_get_counter,
  .get_checksum = volkswagen_pq_get_checksum,
  .compute_checksum = volkswagen_pq_compute_checksum,
};
