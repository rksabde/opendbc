# Volvo SPA: what is on FlexRay, what is on CAN, and where this port connects

This file records what the Volvo port in this folder is based on. Nothing in the port has been run in a car.
No CAN log from an XC90 was available. Every signal comes from public sources, and each one is marked below.

## Short answer

- A harness at the windshield camera, which is where comma harnesses normally go, cannot work on SPA cars.
  The camera and the driver assistance computer (ASDM) talk over FlexRay. A panda only speaks CAN and CAN FD.
- The steering rack is not on FlexRay. It is on a CAN bus that Volvo calls the chassis CAN (later "MID CAN").
  Its only partner for lane centering on that bus is the chassis computer: VDDM on early cars, VCU1 from model year 2020.
- So steering can be reached with a harness that splits the chassis CAN between VCU1 and the steering rack.
  One community fork (StarPilot) steers a 2024 S60 Recharge this way. This port uses the same messages.
- Acceleration and braking requests stay on FlexRay. This port leaves the stock adaptive cruise control in charge.
- Whether a 2022 XC90 uses the same chassis CAN messages as the 2024 S60 Recharge is not known. It has to be checked in the car.

## Bus map

Names are the ones in Volvo's wiring diagrams. "2016 set" is Volvo R&D wiring document 31839941 issue 028 (sheets edited 2012 to January 2016),
as posted on a forum. "2024 set" is Volvo Service Journal SJ1001861, "Aftermarket Wiring Diagrams SPA MY24", as filed with NHTSA.

| Module | 2016 set (XC90) | 2024 set (XC90, XC60, S90, V90, S60, V60) |
|---|---|---|
| Steering rack (EPAS / PSCM) | Chassis CAN, pins B2 (high) and B3 (low). No FlexRay | MID CAN (chassis CAN), same pins B2 and B3. No FlexRay |
| Chassis computer | VDDM: FlexRay backbone and chassis CAN | VCU1: FlexRay backbone, FlexRay "Front 2", MID CAN (pins A20 high, A19 low), propulsion CAN |
| Driver assistance computer (ASDM) | FlexRay backbone, two safety CANs. It is the windshield unit itself | FlexRay backbone, FlexRay "Mid 2" to the camera, CAN to the front radar, two safety CANs |
| Windshield camera (FLC2) | part of the ASDM | FlexRay "Mid 2" only |
| Brakes | done by the VDDM | BCM2: FlexRay "Front 2" with VCU1, and a private CAN to the rear motor inverter |
| Also on the chassis CAN | suspension module, steering wheel module, steering column lock, engine computer on some variants | steering wheel module, steering angle sensor, steering column lock, gear selector |
| Engine and gearbox | propulsion CAN | Front CAN 1 (propulsion CAN) |
| Central module (CEM), cluster, infotainment, airbags | FlexRay backbone | FlexRay backbone |
| OBD socket | Ethernet, and the propulsion CAN | Ethernet, and diagnostic CAN to the gateway and to VCU1 |

Volvo service bulletins show VCU1 fitted to the XC90 from structure week 201917 (model year 2020), and the gateway module VGM from model year 2023.
A 2022 XC90 therefore has VCU1, like the 2024 set, but no wiring diagram for that exact year was found.

How sure each point is:

| Claim | Basis | Confidence |
|---|---|---|
| The steering rack is CAN only in the 2024 set | Read directly in the MID CAN sheet (page 44 of 177) | High |
| The steering rack is CAN only in the 2016 set | Read by a research pass in the forum copy of the document | High for the content, medium for where the document came from |
| A 2022 XC90 has the rack on the chassis CAN with VCU1 | The rack is on the same pins in 2016 and 2024, and VCU1 is in XC90 bulletins from 2020 | Medium to high, not seen directly |
| The chassis CAN messages of a 2022 XC90 match the 2024 S60 Recharge | Same platform and same chassis computer. No XC90 data | Unknown |
| The camera link is FlexRay only | 2024 set, FlexRay "Mid 2" sheet | High for 2024, unknown for 2022 |
| Bit rate of the chassis CAN, and whether it is CAN FD | Not stated anywhere. Older Volvos use 500 kbit/s | Unknown |

## Where the harness goes

The harness opens the chassis CAN so that VCU1 is on one side and the steering rack on the other.
It can be made at the VCU1 connector or at the steering rack connector. The port works with either, because it only reads
messages from VCU1 on one side and messages from the steering rack on the other.

| Panda bus | Connects to | Used for |
|---|---|---|
| 0 | chassis CAN, VCU1 side | speed, brake pedal, Pilot Assist state, gear, the stock steering angle request |
| 2 | chassis CAN, steering rack side | steering angle, driver steering input. openpilot sends the steering angle request here |
| 1 | propulsion CAN | accelerator pedal |

There is no harness for this in the comma shop. The connector part numbers were not found.
A split CAN bus needs a termination resistor on each half. Whether the harness has to add one depends on where the bus is opened, and has to be measured.

## What the port does

VCU1 sends the lane centering request to the steering rack in several messages. One of them, `LCA_5` (0x67), carries the requested steering wheel angle.
The others say whether the rack should follow it and with how much force.

- The panda stops `LCA_5` from VCU1 reaching the rack. openpilot sends each one on, once, unchanged except for the angle.
  The wheel speeds, counter and checksum in the message are kept. The checksum does not cover the angle.
- While openpilot is steering, the angle is openpilot's. Otherwise the angle from VCU1 is sent on, so Pilot Assist steers as it normally does.
- openpilot sends nothing else. It cannot switch lane centering on in the rack. The rack only follows the angle while Volvo's own Pilot Assist is on,
  and the stock system keeps deciding how easily the driver can overrule the steering.
- openpilot engages when Pilot Assist engages, and disengages when it disengages or the brake pedal is pressed.

The panda safety code (`opendbc/safety/modes/volvo.h`) allows two kinds of `LCA_5`:
an angle that VCU1 itself sent in one of its last six messages, or, only while controls are allowed, an angle inside the same lateral acceleration and jerk limits
that the Tesla and BYD ports use.

## Signals and where they come from

None of these was verified on an XC90. "StarPilot" means the DBC files and code in github.com/StarPilotHQ/StarPilot (commit a732375f),
which its authors made from their own drives in a 2024 S60 Recharge, an XC40 Recharge and a Polestar 2. Those drives are not public.

| Signal | Message | Source | Open questions |
|---|---|---|---|
| Vehicle speed | `SPEED` 0x60, bus 0 | StarPilot. Scale fitted against GPS speed | none known |
| Steering angle | `PSCM` 0x16, bus 2 | StarPilot | none known |
| Driver steering input | `DRIVER_INPUT` 0x15, bus 2 | StarPilot | unit and sign. The threshold of 5 for "driver is steering" is a guess from their notes |
| Brake pedal | `LCA_2` 0x69, bus 0 | StarPilot | none known |
| Pilot Assist engaged | `LCA_2` 0x69, bus 0 | StarPilot | whether it drops when only the steering part of Pilot Assist pauses |
| Stability control active | `LCA_2` 0x69, bus 0 | StarPilot | seen in few events |
| `LCA_2` checksums | bytes 2 and 6 | StarPilot's fit, rewritten here as CRC-8 SAE J1850. The two agree for every value their logs had | the constants could differ on other cars. One bit of byte 3 did not fit |
| Gear | `GEAR_POSITION` 0x80, bus 0 | StarPilot | none known |
| Accelerator pedal | `ECM_1` 0x25, bus 1 | StarPilot (S60 Recharge) | may differ by engine |
| Steering angle request | `LCA_5` 0x67 | StarPilot | the value 0x3A00 is seen while lane centering is off. Its meaning is a guess |

Not found on these buses, or not found at all: cruise set speed, cruise buttons, blinkers, doors, seatbelt, steering fault state, blind spot warnings.
openpilot needs the doors, seatbelt and blinkers before this port can leave dashcam mode. A capture of a 2021 V90 found the turn signals
on the body CAN behind the central module (message 0x040), which this harness does not reach.

## What is on FlexRay

- The camera to ASDM link, and the lane centering request from ASDM to VCU1. This port does not need them, because it replaces the angle after VCU1.
- The acceleration and braking requests from the adaptive cruise control: ASDM to VCU1 on the backbone, and VCU1 to the brake module on "Front 2".
  openpilot longitudinal control needs a FlexRay bridge.
- Probably the door, seatbelt and blinker states that the central module shares with the other computers.

The FlexRay settings of SPA cars are not public: bit rate, cycle length, number of slots, and which slot carries what. They have to be recorded from a car first.

## A FlexRay bridge for longitudinal control

This is a design for a follow-up. None of it was built.

**Where.** In the single FlexRay pair between the ASDM and the backbone. The bridge then changes what the ASDM asks for, and VCU1 still decides what the
brakes and engine do. The link between VCU1 and the brake module is the wrong place: a bridge there would command the brakes directly.

**What it does.** FlexRay gives each computer fixed time slots. A device cannot add a message of its own; it has to sit in line and take over the sender's slots.
The bridge has two FlexRay ports. It passes every frame through in both directions, and for the few frames that carry the ASDM's acceleration request
it swaps the payload and recalculates the frame CRC, and any checksum and counter inside the payload.

**What has to be found in the car first.**
1. The cluster settings, by recording the backbone with a FlexRay analyzer.
2. Which slots the ASDM sends, and which of those change with the cruise control request: record with the cruise control off, holding speed, braking and accelerating.
3. Whether the ASDM is one of the computers that starts the cluster. If it is, the bridge must keep both sides in sync during startup.
4. Whether the payloads carry a checksum and counter like the CAN messages do.

**Hardware that has done this.**
- comma's 2020 proof of concept on an Audi Q8: an FPGA (Cyclone 10 LP) with FlexRay transceivers, changing a steering frame as it passed and recalculating the CRC.
- `dynm/pico-flexray`: a Raspberry Pi Pico 2 with two FlexRay transceivers (TJA1082 or TLE9222). A sunnypilot fork uses it to steer BMWs and calls it very experimental.
- A microcontroller with two FlexRay controllers (for example Infineon AURIX TC3xx) would do the same with a delay of one cycle per frame.

**How the panda stays in charge.**
- The bridge copies the ASDM's original frames to the panda as CAN FD frames, and takes the replacement frames from the panda the same way.
  The panda safety code then works as for any CAN FD car: it limits acceleration, requires controls allowed, and can let stock emergency braking through.
- The bridge only ever changes a fixed list of slots. If no valid replacement arrives in time, it passes the ASDM's own frame.
- A relay joins the ASDM straight to the backbone when the bridge has no power, like the relay in a comma harness box.
  At FlexRay speeds the relay and wiring must keep the line impedance, or the whole backbone is disturbed.

## Checks in the car before anything is sent

1. Find the steering rack connector and confirm CAN on pins B2 and B3: about 60 ohm between them with the car off, CAN traffic with it on. Measure the bit rate.
2. Record the chassis CAN without splitting it. Compare the message list with `opendbc/dbc/volvo_spa_2024_chassis.dbc`: 0x15, 0x16, 0x60, 0x67, 0x69, 0x80 should be there.
3. In that recording, check each signal in the table above against what the car was doing, and check that `LCA_2` passes its two checksums.
4. Record with Pilot Assist on and off, and with the hands on and off the wheel. Check that the angle in `LCA_5` follows the steering angle while Pilot Assist steers.
5. Only then split the bus, with the port still in dashcam mode, and check that the car behaves as before with openpilot sending every `LCA_5` on unchanged.

## Sources

- Volvo Service Journal SJ1001861, wiring diagrams SPA MY24: https://static.nhtsa.gov/odi/tsbs/2023/MC-10236766-9999.pdf
- Volvo R&D wiring set 31839941 issue 028, forum copy: https://www.swedespeed.com/attachments/wiring_diagram_sv90-pdf.211408/
- VCU1 on the XC90 from model year 2020: https://static.nhtsa.gov/odi/tsbs/2020/MC-10181044-9999.pdf
- VGM on the XC90 from model year 2023: https://static.nhtsa.gov/odi/tsbs/2024/MC-11009509-0001.pdf
- StarPilot Volvo port: https://github.com/StarPilotHQ/StarPilot (opendbc_repo/opendbc/car/volvo, opendbc_repo/opendbc/dbc/volvo_mid_1.dbc)
- 2021 V90 body CAN capture: https://github.com/lukas-ad-dev/volvo-can-diagnostics
- comma, FlexRay on an Audi Q8: https://blog.comma.ai/hacking-an-audi-performing-a-man-in-the-middle-attack-on-flexray/
- pico-flexray: https://github.com/dynm/pico-flexray
