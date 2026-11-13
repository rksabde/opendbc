from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volvo"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvo)]

    # Not validated in a car
    ret.dashcamOnly = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4

    # The radar and the acceleration and brake requests are not on the CAN buses the harness connects to
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False

    return ret
