# Hardware Requirements Specification

| Item | Value |
|---|---|
| Document | HRS, EV traction inverter and VCU control hardware |
| Status | Draft, for review |
| Parent | [01 System Requirements](01_system_requirements.md) |
| Design | [06 Hardware Design and Schematics](06_hardware_design.md) |
| Verification | [07 Hardware Verification Cases](07_hardware_verification_cases.md), [08 Hardware Verification Results](08_hardware_verification_results.md) |

## 1. Scope and conventions

These requirements cover the control electronics (MCU, power supplies,
sensing, gate drive, vehicle interfaces) and the inverter's high-voltage
power path. IDs have the form `HWR-<area>-nnn`. The **Traces to** column gives
the parent system requirements. Every hardware requirement is verified by at
least one case in [07](07_hardware_verification_cases.md); `make trace`
checks this.

Values in parentheses refer to firmware parameters in `config/*.h` where the
hardware and software must agree. `make hw-report` checks that agreement.

## 2. Power supply (PWR)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-PWR-001 | The control electronics shall operate from the 12 V vehicle supply between 9 V and 16 V, survive 24 V for 60 s (jump start) and −14 V for 60 s (reverse polarity) without damage, and withstand a load dump clamped to 35 V (ISO 16750-2, test B). | SYS-060 |
| HWR-PWR-002 | The power supply shall provide 5 V (logic and peripherals), 3.3 V (MCU I/O), a separate 3.3 V analog supply used as ADC reference with ±0.5 % tolerance, and the MCU core supply according to the MPC5744P data sheet. | SYS-060 |
| HWR-PWR-003 | Each accelerator pedal track shall be supplied from its own 5 V sensor supply, protected against short circuit to ground and to battery. | SYS-040 |
| HWR-PWR-004 | The phase current sensors shall be supplied from a 5.000 V supply with ±0.5 % tolerance (ratiometric sensors: supply error is an offset error). | SYS-041 |
| HWR-PWR-005 | In the worst case, each regulator shall be loaded to at most 70 % of its rated output current, and each isolated gate supply to at most 70 % of its rated power. | SYS-060 |

## 3. Microcontroller (MCU)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-MCU-001 | The controller shall be an NXP MPC5744P clocked from a 40 MHz crystal with a total tolerance of ±100 ppm or better (temperature and aging included). | SYS-060, SYS-050 |
| HWR-MCU-002 | A JTAG connector shall give access for programming and debugging. | SYS-060 |
| HWR-MCU-003 | A supervisor shall hold the MCU in reset while the 3.3 V supply is out of tolerance. All PWM, gate-enable, contactor and discharge outputs shall have pull-downs so they are off while the MCU is in reset or unpowered. | SYS-046, SYS-047 |

## 4. High-voltage path (HV)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-HV-001 | The HV− contactor, HV+ contactor and precharge relay (in the battery junction box) shall be driven by separate MCU-controlled low-side outputs; all coils shall be de-energised when the outputs are off. | SYS-030, SYS-047 |
| HWR-HV-002 | The precharge resistor shall charge the DC link to 95 % of the maximum battery voltage within one third of the precharge timeout (`PRECHARGE_TIMEOUT_MS`) over component tolerances, with a peak current ≤ 10 A, and shall be rated for 5 consecutive precharges. | SYS-030 |
| HWR-HV-003 | The active discharge shall reduce the DC link from the overvoltage trip level (`HV_OV_TRIP_V`) to below 60 V within 1 s, with the resistor loaded to at most 50 % of its pulse energy rating. | SYS-031 |
| HWR-HV-004 | A passive bleeder shall reduce the DC link from the overvoltage trip level to below 60 V within 120 s without any supply, with each resistor at ≤ 50 % of its power rating and ≤ 75 % of its voltage rating. | SYS-031 |
| HWR-HV-005 | The DC link capacitance shall be 500 µF ±10 % with a voltage rating ≥ 1.05 × the overvoltage trip level. | SYS-061 |

## 5. Power stage and gate drive (PS, GD)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-PS-001 | The power module shall have a blocking voltage ≥ 1.6 × the overvoltage trip level (switching overshoot margin) and a nominal current ≥ 1.4 × the software overcurrent trip (`INV_I_TRIP_A`). | SYS-001, SYS-041 |
| HWR-PS-002 | The PWM period and dead time shall be exact integer counts of the motor-control clock, and the dead time (`INV_DEADTIME_NS`) shall exceed the worst-case switch turn-off time plus driver propagation skew. | SYS-061 |
| HWR-GD-001 | Each of the six switches shall have its own reinforced isolated gate driver with an isolated +15 V / −5 V supply. | SYS-061, SYS-048 |
| HWR-GD-002 | Each gate driver shall detect desaturation, turn its switch off softly, and assert an open-drain fault output. The six fault outputs shall be wired-OR into the FlexPWM_0 FAULT0 input (active low, pulled up). | SYS-062 |
| HWR-GD-003 | A common gate-enable signal with a pull-down shall disable all gate drivers. | SYS-047 |
| HWR-GD-004 | Each gate driver shall use a negative turn-off voltage and an active Miller clamp to prevent parasitic turn-on. | SYS-061 |

## 6. Sensing (SNS)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-SNS-001 | Phase currents shall be measured in all three phases with an ADC range of at least ±1.2 × `INV_I_TRIP_A`, a resolution ≤ 1 A/count, and an anti-alias filter whose phase lag at the current loop bandwidth (`CURRENT_LOOP_BW_HZ`) is ≤ 5°. The firmware scaling (`CUR_A_PER_COUNT`) shall match the hardware within 0.5 %. | SYS-041, SYS-001 |
| HWR-SNS-002 | The worst-case zero-current offset at the ADC, including sensor offset and supply tolerance, shall be ≤ 60 % of the calibration limit (`INV_I_OFFSET_MAX_A`), so that start-up calibration does not reject a healthy sensor. Gain error ≤ 3 %. | SYS-041 |
| HWR-SNS-003 | The DC link voltage shall be measured through a reinforced isolated amplifier with a full scale ≥ 1.15 × the overvoltage trip level and a resolution ≤ 0.5 V/count; the firmware scaling (`VDC_V_PER_COUNT`) shall match within 0.5 %. | SYS-041, SYS-048 |
| HWR-SNS-004 | Inverter (cold plate) and motor winding temperatures shall be measured with NTCs to a resolution ≤ 0.5 °C/count at the trip temperatures, and the valid range −40…175 °C shall be distinguishable from open and short circuit (50 mV thresholds), including 1 % component tolerances. | SYS-041, SYS-045 |
| HWR-SNS-005 | The resolver interface shall provide the excitation drive and a resolver-to-digital converter delivering a 16-bit position over SPI, and a fault output for loss of signal or tracking error. | SYS-063 |
| HWR-SNS-006 | At maximum duty cycle, the current sample shall be taken at least 1 µs after the last switching edge. | SYS-061 |

## 7. Vehicle interfaces (IF)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-IF-001 | Each pedal track shall be divided into the ADC range so that both out-of-range thresholds lie within 3…97 % of full scale, a short to the track's own supply is detected as out of range, and a short to 16 V injects ≤ 50 % of the MCU pin's injection current limit. The firmware scaling (`APPS_V_PER_COUNT`) shall match within 0.5 %. | SYS-040 |
| HWR-IF-002 | The six switch inputs (key, brake, P, R, N, D) shall read HIGH for inputs ≥ 8 V and LOW for inputs ≤ 2 V, provide ≥ 2 mA contact wetting current at 9 V, and withstand 24 V and −14 V without exceeding component ratings. | SYS-020, SYS-023 |
| HWR-IF-003 | Five protected low-side outputs (HV−, precharge, HV+, discharge drive, spare) shall switch ≥ 2 A continuous, clamp the inductive turn-off voltage actively (fast contactor release), and be short-circuit protected. | SYS-030, SYS-031 |
| HWR-IF-004 | The CAN interface shall use an ISO 11898-2 transceiver with ESD protection, a common-mode choke and optional split termination. | SYS-050, SYS-051 |
| HWR-IF-005 | The FlexCAN bit timing (500 kbit/s, sample point 85…90 %) shall tolerate the clock source tolerance according to ISO 11898-1. | SYS-050 |

## 8. Environment and isolation (ENV, ISO)

| ID | Requirement | Traces to |
|---|---|---|
| HWR-ENV-001 | The control electronics shall operate at ambient temperatures from −40 °C to +105 °C, using AEC-Q100 / AEC-Q200 qualified components. | SYS-060 |
| HWR-ENV-002 | The system shall meet CISPR 25 class 3 emissions and ISO 11452-2 / -4 immunity limits agreed with the vehicle manufacturer. | SYS-060 |
| HWR-ENV-003 | Connector pins shall withstand ±8 kV contact discharge according to ISO 10605 without damage. | SYS-060 |
| HWR-ENV-004 | The assembly shall withstand vibration and mechanical shock according to ISO 16750-3 for the mounting location. | SYS-060 |
| HWR-ISO-001 | HV and LV circuits shall be separated by reinforced isolation with creepage and clearance distances according to IEC 60664-1 for the maximum working voltage, and shall pass a dielectric withstand and insulation resistance test according to ISO 6469-3. | SYS-048 |
