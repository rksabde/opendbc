from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams


def get_safety_CP():
  from opendbc.car.volvo.interface import CarInterface
  return CarInterface.get_non_essential_params("VOLVO_XC90_MK2")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0.0
    self.lca_5_ts_last = 0

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # The panda blocks the stock LCA_5 from reaching the steering rack, so each one that arrives is sent on once.
    # The steering rack only follows the angle in it while VCU1 enables lane centering in its other messages.
    if CS.lca_5_ts != self.lca_5_ts_last:
      self.lca_5_ts_last = CS.lca_5_ts
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams, self.VM)
      can_sends.append(volvocan.create_steering_control(self.packer, CS.lca_5, self.apply_angle_last, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
