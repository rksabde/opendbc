from opendbc.car.crc import CRC8J1850


def volvo_checksum(address: int, sig, d: bytearray) -> int:
  # LCA_2 has two checksums. Each is a CRC over a constant, a counter and the values of the signals it protects
  if sig.name == "CHECKSUM_1":
    dat = [0xEF, d[5] & 0xF, d[5] >> 7, (d[5] >> 6) & 0x1, (d[5] >> 4) & 0x3]
  else:
    dat = [0x04, d[1] & 0xF, (d[1] >> 4) & 0x7, d[4], d[3], d[0]]

  crc = 0xFF
  for byte in dat:
    crc = CRC8J1850[crc ^ byte]
  return crc ^ 0xFF


def create_steering_control(packer, stock_lca_5: dict, apply_angle: float, lat_active: bool):
  # The wheel speeds, counter and checksum are sent on unchanged, the checksum does not cover the angle.
  # The angle VCU1 requests is also sent on unchanged while openpilot is not steering.
  values = dict(stock_lca_5)
  if lat_active:
    values["LCA_5_STEER"] = apply_angle
  return packer.make_can_msg("LCA_5", 2, values)
