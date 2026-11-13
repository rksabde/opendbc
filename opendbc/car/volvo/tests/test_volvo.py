import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC
from opendbc.car.volvo.volvocan import volvo_checksum

LCA_5 = 0x67
LCA_2 = 0x69

# LCA_5 with lane centering off, from the test vectors of the StarPilot checksum helper
STOCK_LCA_5 = [bytes.fromhex(dat) for dat in ("8000324f0000ba00", "8000898f0000ba00", "8000e0cf0000ba00", "8000061f0000ba00")]

# The LCA_2 checksums as StarPilot fitted them to logs: bit i of the checksum is the parity of the frame bytes under mask i.
# The fits only hold for values the logs had: BYTE_0 from 0x18 to 0x1B, bit 0 of byte 3, and no UNKNOWN_BITS_45 in byte 5.
CHECKSUM_1_MASKS = {0: [0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00], 5: [0x83, 0x86, 0xCF, 0xCD, 0x09, 0x02, 0x44, 0x89]}
CHECKSUM_2_MASKS = {0: [0x01, 0x0A, 0x09, 0x03, 0x0B, 0x02, 0x08, 0x08], 1: [0x3C, 0x69, 0x7E, 0x51, 0x1E, 0x2D, 0x4B, 0x16],
                    3: [0x00, 0x02, 0x03, 0x03, 0x00, 0x02, 0x01, 0x00], 4: [0x03, 0x07, 0x0D, 0x19, 0x30, 0x60, 0xC0, 0x81]}


def parity_checksum(masks: dict[int, list[int]], dat: bytes) -> int:
  return sum((sum((dat[byte] & mask[i]).bit_count() for byte, mask in masks.items()) & 1) << i for i in range(8))


class TestVolvoChecksum(unittest.TestCase):
  def setUp(self):
    self.sigs = CANPacker(DBC[CAR.VOLVO_XC90_MK2][Bus.main]).dbc.name_to_msg["LCA_2"].sigs

  def test_lca_2_checksum_1(self):
    for byte_5 in range(0x100):
      if byte_5 & 0x30 == 0:
        dat = bytes([0x18, 0, 0, 0, 0, byte_5, 0, 0])
        self.assertEqual(parity_checksum(CHECKSUM_1_MASKS, dat), volvo_checksum(LCA_2, self.sigs["CHECKSUM_1"], bytearray(dat)))

  def test_lca_2_checksum_2(self):
    for byte_0 in range(0x18, 0x1C):
      for byte_1 in range(0x80):
        for byte_3 in range(2):
          for byte_4 in range(0x100):
            dat = bytes([byte_0, byte_1, 0, byte_3, byte_4, 0, 0, 0])
            self.assertEqual(parity_checksum(CHECKSUM_2_MASKS, dat), volvo_checksum(LCA_2, self.sigs["CHECKSUM_2"], bytearray(dat)))


class TestVolvoSteeringControl(unittest.TestCase):
  def setUp(self):
    self.CI = CarInterface(CarInterface.get_non_essential_params(CAR.VOLVO_XC90_MK2))
    self.nanos = 0

  def _step(self, can_msgs, lat_active=False):
    self.nanos += 10_000_000
    self.CI.update([(self.nanos, can_msgs)])
    CC = structs.CarControl()
    CC.enabled = CC.latActive = lat_active
    return self.CI.apply(CC.as_reader(), self.nanos)[1]

  def test_stock_message_sent_on(self):
    # nothing is sent before the first stock message
    self.assertEqual([], self._step([]))

    # every stock message is sent on to the steering rack once and unchanged while not steering
    for dat in STOCK_LCA_5:
      self.assertEqual([(LCA_5, dat, 2)], self._step([(LCA_5, dat, 0)]))
      self.assertEqual([], self._step([]))

  def test_steering_angle_replaced(self):
    # only the angle changes while steering: here to 0 deg, the steering angle the car state starts with
    self._step([])
    for dat in STOCK_LCA_5:
      self.assertEqual([(LCA_5, dat[:6] + bytes([0x80, 0x00]), 2)], self._step([(LCA_5, dat, 0)], lat_active=True))


if __name__ == "__main__":
  unittest.main()
