# System Requirements Specification

| Item | Value |
|---|---|
| Document | SyRS, EV traction inverter and vehicle control unit |
| Project | MPC5744P EV drive firmware |
| Status | Draft, for review |
| Related documents | [02 Software Requirements](02_software_requirements.md), [03 Software Verification Cases](03_software_verification_cases.md), [04 Software Verification Results](04_software_verification_results.md) |

## 1. Scope

This document specifies the electric drive system of a battery electric
passenger vehicle, as seen from the vehicle. The system consists of:

* a three-phase IPMSM traction motor with a resolver,
* a two-level IGBT/SiC traction inverter with gate drivers, phase current
  sensors, a DC link voltage sensor and temperature sensors,
* one NXP MPC5744P microcontroller that performs both the inverter control and
  the vehicle control unit (VCU) function,
* interfaces to the driver (key, accelerator pedal, brake switch, gear
  selector), to the HV battery (contactors, precharge, BMS over CAN) and to the
  rest of the vehicle (CAN).

Out of scope: the battery management system, the hydraulic brake system,
steering, charging, thermal management actuators and the DC/DC converter.

**Example values.** Numerical values describe an example 100 kW / 350 V drive.
They are kept in `config/ev_config.h` and must be replaced together with this
document for a real vehicle.

## 2. Definitions

| Term | Meaning |
|---|---|
| ASC | Active short circuit: all low-side switches on, all high-side switches off |
| BMS | Battery management system |
| DC link | Inverter input capacitor bank, connected to the battery through contactors |
| Freewheel | All six inverter switches off |
| HV | High voltage (battery and DC link) |
| IPMSM | Interior permanent magnet synchronous machine |
| Key cycle | Key off followed by key on |
| Motor speed | Mechanical rotor speed in rpm; positive = forward |
| Safe state | Inverter not producing commanded torque (freewheel or ASC) |
| Torque | Motor shaft torque; positive accelerates the vehicle forward |

## 3. Requirement notation

Each requirement has a unique ID `SYS-nnn`. "Shall" denotes a mandatory
requirement. Each requirement is refined by at least one software
requirement (SWR) in [02_software_requirements.md](02_software_requirements.md).

## 4. Functional requirements

### 4.1 Propulsion

| ID | Requirement | Rationale |
|---|---|---|
| SYS-001 | The system shall produce forward motor torque proportional to the accelerator pedal position, up to 230 Nm and limited to 100 kW mechanical power. | Basic propulsion. |
| SYS-002 | The system shall operate up to a motor speed of 12000 rpm from a 350 V DC link, keeping the requested torque where the available voltage and current allow it. | Vehicle top speed requires field weakening. |
| SYS-003 | The system shall not request more battery current than the discharge and charge current limits reported by the BMS. | Battery protection. |
| SYS-004 | In reverse, the system shall limit torque to 40 % of the forward maximum and motor speed to 2500 rpm. | Low-speed manoeuvring only. |
| SYS-005 | Changes of the motor torque shall be rate limited, with a slower rate around zero torque. | Drivability, driveline backlash. |

### 4.2 Regenerative braking

| ID | Requirement | Rationale |
|---|---|---|
| SYS-010 | With the accelerator released, the system shall apply a coast regeneration torque of 30 Nm; with the brake pedal pressed, 100 Nm; in both cases limited to 150 Nm and 60 kW. | Energy recovery, one-pedal feel. |
| SYS-011 | Regenerative torque shall fade out between 800 rpm and 200 rpm and shall not be applied when the vehicle rolls against the selected direction. | Prevent the vehicle from being driven backwards by regeneration. |

### 4.3 Driver interface

| ID | Requirement | Rationale |
|---|---|---|
| SYS-020 | The system shall provide the gear positions P, R, N and D. Engaging D or R shall require the brake pedal pressed and the vehicle at standstill. | Prevent unintended movement. |
| SYS-021 | Selecting P while moving, or changing between D and R while moving, shall be rejected. | Driveline protection, unintended direction. |
| SYS-022 | When brake and accelerator are pressed together, the brake shall take priority and drive torque shall be removed. | Brake override (stuck pedal mitigation). |
| SYS-023 | An implausible gear selector signal shall not change the engaged gear. | Single-point fault of the selector. |

### 4.4 High-voltage management

| ID | Requirement | Rationale |
|---|---|---|
| SYS-030 | On key on with the BMS ready, the system shall connect the battery through a precharge circuit and close the main contactor only when the DC link has reached 95 % of the battery voltage, within 3 s. | Limit inrush current, contactor life. |
| SYS-031 | On key off, the system shall open the contactors and discharge the DC link to below 60 V within 5 s. | Electrical safety after shutdown. |
| SYS-032 | A key off while the motor turns faster than 500 rpm shall not disconnect HV or end motor control until the speed has dropped below that value. | Avoid uncontrolled machine at speed. |

### 4.5 Safety

| ID | Requirement | Rationale |
|---|---|---|
| SYS-040 | A single fault of the accelerator pedal sensor shall not cause unintended torque. If only one sensor track has failed, the vehicle shall remain drivable at reduced torque (limp home). | Unintended acceleration hazard. |
| SYS-041 | The system shall detect phase overcurrent, DC link over- and undervoltage, inverter and motor over-temperature, motor overspeed, rotor position sensor faults and current sensor faults, and shall remove torque. | Component protection, loss of control. |
| SYS-042 | When torque is removed, the inverter shall go to freewheel at low speed and to active short circuit at high speed, where back-EMF exceeds the DC link voltage. After a hardware overcurrent, all switches shall be off. | Prevent uncontrolled regeneration into the battery; protect a shorted bridge. |
| SYS-043 | Loss of BMS communication for 300 ms, a BMS fault or a precharge failure shall cause HV disconnection. | Battery safety. |
| SYS-044 | Detected faults shall remain active until the next key cycle in standby, even if the cause disappears. | Avoid intermittent re-enabling. |
| SYS-045 | Torque shall be reduced linearly when the inverter or motor approaches its temperature limit or when the DC link voltage approaches its lower limit. | Graceful degradation instead of abrupt cut-off. |
| SYS-046 | Failure of the control software to execute its periodic tasks at the specified rates shall lead to a microcontroller reset within 50 ms. | Program flow / temporal monitoring. |
| SYS-047 | At power-up and in standby, all contactors, the discharge circuit, the gate drivers and the PWM outputs shall be inactive. | Safe initial state. |

### 4.6 Communication

| ID | Requirement | Rationale |
|---|---|---|
| SYS-050 | The system shall receive battery status and current limits from the BMS on CAN (500 kbit/s) and shall reject corrupted or stale messages. | Data integrity. |
| SYS-051 | The system shall transmit its state, gear, speed, torque command, fault reaction, fault set, DC link voltage and temperatures on CAN every 10 ms. | Instrument cluster, diagnostics. |

## 5. Platform and design constraints

| ID | Requirement | Rationale |
|---|---|---|
| SYS-060 | The controller shall be an NXP MPC5744P running at 200 MHz. | Platform decision (lockstep core, motor control peripherals). |
| SYS-061 | The inverter shall switch at 10 kHz with center-aligned complementary PWM and 1.5 µs dead time; phase currents shall be sampled synchronously to the PWM. | Losses vs. current ripple; shoot-through protection. |
| SYS-062 | A hardware overcurrent / desaturation signal from the gate drivers shall disable the PWM outputs without software involvement. | Reaction time below what software can guarantee. |
| SYS-063 | Rotor position shall be measured with a resolver through a resolver-to-digital converter. | Absolute position at standstill, robustness. |
| SYS-064 | Software shall be developed in C with hardware-dependent code separated from the application, so that the application can be verified without target hardware. | Testability, portability. |

## 6. Open points

* A hazard analysis and risk assessment (ISO 26262-3) has not been performed.
  Safety goals and ASIL ratings therefore aren't assigned. SYS-040 to SYS-047
  are engineering requirements, not derived safety requirements.
* Requirements for FCCU handling, lockstep and memory fault reactions,
  start-up self-tests, contactor weld detection, insulation monitoring and
  E2E protection with CRC are not yet specified (see README, "Missing for a
  production vehicle").
