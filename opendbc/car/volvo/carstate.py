import copy

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import DBC, CarControllerParams as CCP

GearShifter = structs.CarState.GearShifter

GEAR_MAP = {
  0: GearShifter.park,
  1: GearShifter.reverse,
  2: GearShifter.neutral,
  3: GearShifter.drive,
  4: GearShifter.drive,  # B, stronger regenerative braking
}

# matches the vehicle moving threshold in opendbc/safety/modes/volvo.h
STANDSTILL_THRESHOLD = 0.1  # m/s


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.lca_5 = {}
    self.lca_5_ts = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_pt = can_parsers[Bus.pt]
    cp_eps = can_parsers[Bus.alt]
    ret = structs.CarState()

    # speed
    ret.vEgoRaw = cp.vl["SPEED"]["SPEED"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw <= STANDSTILL_THRESHOLD

    # steering wheel
    ret.steeringAngleDeg = cp_eps.vl["PSCM"]["PSCM_ANGLE_SENSOR"]
    ret.steeringTorque = cp_eps.vl["DRIVER_INPUT"]["STEERING_DRIVER_INPUT"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CCP.STEER_DRIVER_ALLOWANCE, 5)

    # gas / brake
    ret.gasPressed = cp_pt.vl["ECM_1"]["ACCELERATOR_PEDAL_POS"] > 1.0  # percent
    ret.brakePressed = bool(cp.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_B"])
    ret.espActive = bool(cp.vl["LCA_2"]["ESC_ACTUATING"]) and bool(cp.vl["LCA_2"]["ESC_ELIGIBLE"])

    # gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["GEAR_POSITION"]["GEAR_POSITION"]), GearShifter.unknown)

    # cruise state: the steering rack only follows the angle request while Pilot Assist is engaged
    ret.cruiseState.available = True
    ret.cruiseState.enabled = bool(cp.vl["LCA_2"]["PILOT_ASSIST_ENGAGED"])
    ret.cruiseState.standstill = ret.standstill

    # TODO: blinkers, doors, seatbelt, cruise set speed, cruise buttons and the steering fault state
    # are not on the CAN buses the harness connects to, or have not been found yet

    # stock steering request and when it was received, every one is sent on by the car controller
    self.lca_5 = copy.copy(cp.vl["LCA_5"])
    self.lca_5_ts = cp.ts_nanos["LCA_5"]["LCA_5_STEER"]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.main], [], 2),
    }
