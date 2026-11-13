#pragma once

#include "opendbc/safety/declarations.h"

// The panda sits on the chassis CAN between VCU1 and the steering rack:
// bus 0 is VCU1, bus 2 is the steering rack, bus 1 is the propulsion CAN
#define VOLVO_PSCM   0x16U  // from the steering rack: steering angle
#define VOLVO_ECM_1  0x25U  // from the engine: accelerator pedal
#define VOLVO_SPEED  0x60U  // from VCU1: vehicle speed
#define VOLVO_LCA_5  0x67U  // from VCU1: steering angle request, replaced by openpilot
#define VOLVO_LCA_2  0x69U  // from VCU1: brake pedal and Pilot Assist state

// Steering angles are 15 bit signed, big endian, 0.05596 deg per bit
static int volvo_get_angle(const CANPacket_t *msg, uint8_t byte) {
  return to_signed(((msg->data[byte] & 0x7FU) << 8) | msg->data[byte + 1U], 15);
}

static uint32_t volvo_get_checksum(const CANPacket_t *msg) {
  // LCA_2 has two: CHECKSUM_2 in byte 2 and CHECKSUM_1 in byte 6
  return ((uint32_t)msg->data[2] << 8) | msg->data[6];
}

static uint32_t volvo_compute_checksum(const CANPacket_t *msg) {
  // Each LCA_2 checksum is a CRC-8 SAE J1850 over a constant, a counter and the values of the signals it protects
  const uint8_t poly = 0x1DU;

  uint8_t checksum_1 = crc8_update(0xFFU, 0xEFU, poly);
  checksum_1 = crc8_update(checksum_1, msg->data[5] & 0xFU, poly);         // COUNTER_2
  checksum_1 = crc8_update(checksum_1, msg->data[5] >> 7, poly);           // BRAKE_PEDAL_PRESSED_A
  checksum_1 = crc8_update(checksum_1, (msg->data[5] >> 6) & 0x1U, poly);  // BRAKE_PEDAL_PRESSED_B
  checksum_1 = crc8_update(checksum_1, (msg->data[5] >> 4) & 0x3U, poly);
  checksum_1 ^= 0xFFU;

  uint8_t checksum_2 = crc8_update(0xFFU, 0x04U, poly);
  checksum_2 = crc8_update(checksum_2, msg->data[1] & 0xFU, poly);         // COUNTER_1
  checksum_2 = crc8_update(checksum_2, (msg->data[1] >> 4) & 0x7U, poly);  // PILOT_ASSIST_ENGAGED, ESC_ACTUATING, ESC_ELIGIBLE
  checksum_2 = crc8_update(checksum_2, msg->data[4], poly);
  checksum_2 = crc8_update(checksum_2, msg->data[3], poly);
  checksum_2 = crc8_update(checksum_2, msg->data[0], poly);
  checksum_2 ^= 0xFFU;

  return ((uint32_t)checksum_2 << 8) | checksum_1;
}

static void volvo_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Vehicle speed: 0.003977 m/s per bit
    if (msg->addr == VOLVO_SPEED) {
      float speed = (((msg->data[0] & 0x7FU) << 8) | msg->data[1]) * 0.003977;
      safety.vehicle_moving = speed > 0.1;
      UPDATE_VEHICLE_SPEED(speed);
    }

    // Brake pedal and cruise state
    if (msg->addr == VOLVO_LCA_2) {
      safety.brake_pressed = GET_BIT(msg, 46U);  // BRAKE_PEDAL_PRESSED_B
      pcm_cruise_check(GET_BIT(msg, 12U));       // PILOT_ASSIST_ENGAGED
    }

    // Steering angle requested by VCU1
    if (msg->addr == VOLVO_LCA_5) {
      update_sample(&safety.volvo_stock_angle, volvo_get_angle(msg, 6U));
    }
  }

  // Accelerator pedal: 1/256 % per bit
  if (msg->bus == 1U) {
    safety.gas_pressed = (((msg->data[0] & 0x7FU) << 8) | msg->data[1]) > 256U;
  }

  // Steering angle
  if (msg->bus == 2U) {
    update_sample(&safety.angle_meas, volvo_get_angle(msg, 0U));
  }
}

static bool volvo_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits VOLVO_STEERING_LIMITS = {
    .max_angle = 9650,  // 540 deg
    .angle_deg_to_can = 17.869907,
    .frequency = 50U,
  };

  // NOTE: based off VOLVO_XC90_MK2 to match openpilot
  const AngleSteeringParams VOLVO_STEERING_PARAMS = {
    .slip_factor = -0.0005620919174371483,  // calc_slip_factor(VM)
    .steer_ratio = 16.8,
    .wheelbase = 2.984,
  };

  // The TX list dispatches only LCA_5 (0x67, bus 2) to this hook.
  // The steering rack follows the angle in it only while VCU1 enables lane centering in other messages,
  // which openpilot can not send.
  int desired_angle = volvo_get_angle(msg, 6U);

  // An angle from one of the last messages of VCU1 is sent on unchanged, which is always allowed
  bool stock_angle = false;
  for (int i = 0; i < MAX_SAMPLE_VALS; i++) {
    if (desired_angle == safety.volvo_stock_angle.values[i]) {
      stock_angle = true;
    }
  }

  bool violation = false;
  if (stock_angle) {
    // openpilot starts from the current steering angle afterwards
    safety.desired_angle_last = SAFETY_CLAMP(safety.angle_meas.values[0], -VOLVO_STEERING_LIMITS.max_angle, VOLVO_STEERING_LIMITS.max_angle);
  } else {
    // openpilot is steering
    violation = steer_angle_cmd_checks_vm(desired_angle, true, VOLVO_STEERING_LIMITS, VOLVO_STEERING_PARAMS);
  }

  return !violation;
}

static safety_config volvo_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static const CanMsg VOLVO_TX_MSGS[] = {
    {VOLVO_LCA_5, 2, 8, .check_relay = true},
  };

  static const RxCheck volvo_rx_checks[] = {
    {.msg = {{VOLVO_PSCM,  2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_ECM_1, 1, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_SPEED, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_LCA_5, 0, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_LCA_2, 0, 8,  50U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(volvo_rx_checks, VOLVO_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
  .get_checksum = volvo_get_checksum,
  .compute_checksum = volvo_compute_checksum,
};
