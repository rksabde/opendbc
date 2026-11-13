from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig
from opendbc.car.lateral import AngleSteeringLimitsVM


class CarControllerParams:
  STEER_STEP = 2  # LCA_5 is sent at 50 Hz

  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    540,  # deg, 1.5 turns to lock
  )

  # DRIVER_INPUT.STEERING_DRIVER_INPUT threshold, unscaled units. The stock system starts yielding at 5
  STEER_DRIVER_ALLOWANCE = 5


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Pilot Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class VolvoSpaPlatformConfig(PlatformConfig):
  # The chassis DBC is used on both sides of the harness: VCU1 (Bus.main) and the steering rack (Bus.alt)
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.main: 'volvo_spa_2024_chassis',
    Bus.pt: 'volvo_spa_2024_pt',
  })


class CAR(Platforms):
  # Not validated in a car. The signals are from StarPilot's 2024 S60 Recharge, the model years from Volvo wiring documents
  VOLVO_XC90_MK2 = VolvoSpaPlatformConfig(
    [VolvoCarDocs("Volvo XC90 Recharge 2022-24")],
    CarSpecs(wheelbase=2.984, steerRatio=16.8),
  )


# No firmware or CAN fingerprint has been recorded from an XC90 yet
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],
  fw_version_regex=b"",
)


DBC = CAR.create_dbc_map()
