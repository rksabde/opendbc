#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.volvo.carcontroller import get_safety_CP
from opendbc.car.volvo.values import CarControllerParams
from opendbc.car.volvo.volvocan import volvo_checksum
import opendbc.safety.tests.common as common

LCA_5 = 0x67

SPEED_TO_CAN = 1 / 0.003977


class TestVolvoSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  DBC = "volvo_spa_2024_chassis"
  SAFETY_MODEL = CarParams.SafetyModel.volvo

  TX_MSGS = [[LCA_5, 2]]
  RELAY_MALFUNCTION_ADDRS = {2: (LCA_5,)}
  FWD_BLACKLISTED_ADDRS = {0: [LCA_5]}

  MAIN_BUS = 0
  PT_BUS = 1
  EPS_BUS = 2

  STANDSTILL_THRESHOLD = 0.1  # m/s
  GAS_PRESSED_THRESHOLD = 1  # percent

  STEER_ANGLE_MAX = 540  # deg
  DEG_TO_CAN = 17.869907

  # Angle VCU1 requests while lane centering is off, in CAN units
  STOCK_INACTIVE_ANGLE = 0x3A00

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  cnt_angle_cmd = 0

  def setUp(self):
    super().setUp()
    self.packer_pt = common.CANPackerSafety("volvo_spa_2024_pt")
    self.VM = VehicleModel(get_safety_CP())
    # every angle in the common tests is then an openpilot command
    self._reset_stock_angle(self.STOCK_INACTIVE_ANGLE)

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def _safety_max_can(self, max_angle_float, can_offset=0):
    # Matches the C: max_angle_can = (int)(max_angle * angle_deg_to_can + 1.)
    return int(max_angle_float * self.DEG_TO_CAN + 1.) + can_offset

  def _safety_speed(self, speed):
    # match both the CAN encoding and the VEHICLE_SPEED_FACTOR rounding in UPDATE_VEHICLE_SPEED. safety fudges the speed by 1 m/s
    sent_can = round((speed + 1) * SPEED_TO_CAN) / SPEED_TO_CAN
    return round(sent_can * common.VEHICLE_SPEED_FACTOR) / common.VEHICLE_SPEED_FACTOR - 1

  def _lca_5_msg(self, angle_can: int, bus: int):
    values = {"LCA_5_STEER": angle_can / self.DEG_TO_CAN}
    return self.packer.make_can_msg_safety("LCA_5", bus, values)

  def _reset_stock_angle(self, angle_can: int):
    for _ in range(common.MAX_SAMPLE_VALS):
      self.assertTrue(self._rx(self._lca_5_msg(angle_can, self.MAIN_BUS)))

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    # There is no request bit, VCU1 enables lane centering in other messages
    if increment_timer:
      self.safety.set_timer(self.__class__.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self._lca_5_msg(round(angle * self.DEG_TO_CAN), self.EPS_BUS)

  def _angle_meas_msg(self, angle: float):
    values = {"PSCM_ANGLE_SENSOR": angle}
    return self.packer.make_can_msg_safety("PSCM", self.EPS_BUS, values)

  def _lca_2_msg(self, values):
    def fix_checksum(msg):
      # the packer only fills in the first checksum of a message
      addr, dat, bus = msg
      dat = bytearray(dat)
      dat[6] = volvo_checksum(addr, self.packer.dbc.name_to_msg["LCA_2"].sigs["CHECKSUM_1"], dat)
      return addr, bytes(dat), bus
    return self.packer.make_can_msg_safety("LCA_2", self.MAIN_BUS, values, fix_checksum=fix_checksum)

  def _pcm_status_msg(self, enable):
    values = {"PILOT_ASSIST_ENGAGED": enable, "BRAKE_PEDAL_PRESSED_B": self.safety.get_brake_pressed_prev()}
    return self._lca_2_msg(values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_PRESSED_B": brake, "PILOT_ASSIST_ENGAGED": self.safety.get_controls_allowed()}
    return self._lca_2_msg(values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("SPEED", self.MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"ACCELERATOR_PEDAL_POS": gas}
    return self.packer_pt.make_can_msg_safety("ECM_1", self.PT_BUS, values)

  def test_vehicle_moving_threshold(self):
    for speed_can, moving in ((25, False), (26, True)):
      self.assertTrue(self._rx(self._speed_msg(speed_can / SPEED_TO_CAN)))
      self.assertEqual(moving, self.safety.get_vehicle_moving())

  def test_gas_threshold(self):
    for gas_can, pressed in ((256, False), (257, True), (0x7FFF, True), (0, False)):
      self.assertTrue(self._rx(self._user_gas_msg(gas_can / 256)))
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertEqual(pressed, self.safety.get_gas_pressed_prev())

  def test_rx_hook_buses(self):
    # the pedal and steering angle messages have the same layout and are told apart by their bus
    self._reset_angle_measurement(50)
    self.assertTrue(self._rx(self._user_gas_msg(100)))
    self.assertTrue(self._rx(self._speed_msg(10)))
    self.assertEqual(round(50 * self.DEG_TO_CAN), self.safety.get_angle_meas_min())
    self.assertEqual(round(50 * self.DEG_TO_CAN), self.safety.get_angle_meas_max())
    self.assertTrue(self.safety.get_gas_pressed_prev())

    self._reset_angle_measurement(0)
    self.assertTrue(self._rx(self._speed_msg(10)))
    self.assertTrue(self.safety.get_gas_pressed_prev())

  def test_rx_checksums(self):
    # everything in LCA_2 is covered by one of its two checksums, except for the top bit of byte 1 and byte 7
    unprotected_bits = {15, *range(56, 64)}
    for values in ({}, {"PILOT_ASSIST_ENGAGED": 1}, {"BRAKE_PEDAL_PRESSED_B": 1}, {"BYTE_0": 0x18, "COUNTER_1": 6, "COUNTER_2": 12},
                   {"ESC_ACTUATING": 1, "ESC_ELIGIBLE": 1, "UNKNOWN_BYTES_3_4": 0x1234, "UNKNOWN_BITS_45": 2, "BRAKE_PEDAL_PRESSED_A": 1}):
      for bit in range(64):
        with self.subTest(values=values, bit=bit):
          self._reset_safety_hooks()
          self.assertTrue(self._rx(self._lca_2_msg(values)))

          self.safety.set_controls_allowed(True)
          msg = self._lca_2_msg(values | {"PILOT_ASSIST_ENGAGED": 1})
          msg[0].data[bit // 8] ^= 1 << (bit % 8)
          valid = bit in unprotected_bits
          self.assertEqual(valid, self._rx(msg))
          self.assertEqual(valid, self.safety.get_controls_allowed())

  def test_stock_angle(self):
    # Only the angles of the last messages from VCU1 can be sent on without controls allowed
    stock_angles = [self.STOCK_INACTIVE_ANGLE, 700, -5, 5, -200, -190]
    for controls_allowed in (False, True):
      with self.subTest(controls_allowed=controls_allowed):
        self._reset_angle_measurement(0)
        self._reset_speed_measurement(0)
        for angle_can in stock_angles:
          self.assertTrue(self._rx(self._lca_5_msg(angle_can, self.MAIN_BUS)))

        for stock_angle in stock_angles:
          for angle_can in range(stock_angle - 2, stock_angle + 3):
            self.safety.set_controls_allowed(controls_allowed)
            self.safety.set_desired_angle_last(angle_can)
            should_tx = controls_allowed or angle_can == stock_angle
            self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle_can / self.DEG_TO_CAN, True)), angle_can)

        # the oldest one is forgotten with the next message
        self.assertTrue(self._rx(self._lca_5_msg(1000, self.MAIN_BUS)))
        for angle_can in (*stock_angles, 1000):
          self.safety.set_controls_allowed(False)
          self.assertEqual(angle_can != stock_angles[0], self._tx(self._lca_5_msg(angle_can, self.EPS_BUS)), angle_can)

  def test_stock_angle_resets_rate_limit(self):
    # After sending on a stock angle, openpilot has to start from the current steering angle
    for angle_meas, angle_last in ((0, 0), (20, 357), (-20, -357), (600, 9650), (-600, -9650)):
      with self.subTest(angle_meas=angle_meas):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(10)
        self._reset_angle_measurement(angle_meas)
        self.safety.set_desired_angle_last(5000)
        self.assertTrue(self._tx(self._lca_5_msg(self.STOCK_INACTIVE_ANGLE, self.EPS_BUS)))
        self.assertEqual(angle_last, self.safety.safety_a.state.desired_angle_last)

  def test_stock_angle_not_rate_limited(self):
    # messages that send on a stock angle do not count towards the real time limit
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    for _ in range(50):
      self.assertTrue(self._tx(self._lca_5_msg(self.STOCK_INACTIVE_ANGLE, self.EPS_BUS)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

  # There is no request bit: an angle that VCU1 did not request is steering, and needs controls allowed
  def test_angle_cmd_when_disabled(self):
    for controls_allowed in (True, False):
      for angle_meas in np.arange(-90, 91, 10):
        self._reset_angle_measurement(angle_meas)

        for angle_cmd in np.arange(-90, 91, 10):
          self.safety.set_controls_allowed(controls_allowed)
          self._set_prev_desired_angle(angle_cmd)
          self.assertEqual(controls_allowed, self._tx(self._angle_cmd_msg(angle_cmd, False)))

  # Volvo uses get_max_angle_delta_vm and get_max_angle_vm for lateral accel and jerk limits
  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = self._safety_speed(max(speed, 1))
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        max_angle_float = get_max_angle_vm(speed, self.VM, CarControllerParams)

        # at limit (safety tolerance adds 1 CAN unit)
        steer_angle_max_can = round(self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        max_angle_can = min(self._safety_max_can(max_angle_float), steer_angle_max_can)
        self.safety.set_desired_angle_last(sign * max_angle_can)

        self.assertTrue(self._tx(self._angle_cmd_msg(sign * max_angle_can / self.DEG_TO_CAN, True)))

        # 1 unit above limit
        over_can = self._safety_max_can(max_angle_float, 1)
        over_can_clipped = min(over_can, steer_angle_max_can)
        self._tx(self._angle_cmd_msg(sign * over_can_clipped / self.DEG_TO_CAN, True))

        # at low speeds max angle is above STEER_ANGLE_MAX, so adding 1 has no effect
        should_tx = over_can >= steer_angle_max_can
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(sign * over_can_clipped / self.DEG_TO_CAN, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = self._safety_speed(max(speed, 1))
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        max_delta_float = get_max_angle_delta_vm(speed, self.VM, CarControllerParams)

        # Stay within limits
        # Up
        max_delta_can = self._safety_max_can(max_delta_float)
        max_angle_delta = sign * max_delta_can / self.DEG_TO_CAN
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        over_delta_can = self._safety_max_can(max_delta_float, 1)
        max_angle_delta = sign * over_delta_can / self.DEG_TO_CAN
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(sign * over_delta_can)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


if __name__ == "__main__":
  unittest.main()
