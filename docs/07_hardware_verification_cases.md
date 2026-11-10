# Hardware Verification Cases

| Item | Value |
|---|---|
| Document | HVCS, verification cases for the control and power hardware |
| Status | Draft, for review |
| Verifies | [05 Hardware Requirements](05_hardware_requirements.md) |
| Results | [08 Hardware Verification Results](08_hardware_verification_results.md) |

## 1. Verification strategy

| Method | Executed by | Description |
|---|---|---|
| Analysis | `make hw-report` | Design calculation from `tools/hw_design.py` and the firmware constants (compiled from `config/*.h`). Runs without hardware. A result that depends on an unconfirmed data sheet value is reported **PASS\***. |
| Review | Engineer, with checklist | Schematic, layout or qualification review against data sheets and standards. |
| Bench test | Lab, on prototype hardware | Functional and electrical measurement. |
| Environmental test | Accredited lab | Temperature, EMC, ESD, vibration, dielectric tests. |

Analyses come first: they are cheap, and they caught five design errors
before any hardware existed (see [06](06_hardware_design.md)). Bench and
environmental cases confirm the analyses on real hardware and cover what
can't be calculated.

**General bench setup**: prototype control board, programmable 0…30 V / 10 A
LV supply, 4-channel oscilloscope (≥ 200 MHz) with differential and current
probes, DMM, CAN analyser, MPC5744P debugger. **HV tests (BN-008…010) only
by qualified personnel**, with a current-limited HV supply, interlocked
enclosure and discharge verification before touching.

## 2. Analyses

### HVC-AN-001 Phase current measurement range and scaling

| | |
|---|---|
| Verifies | HWR-SNS-001 |
| Method | Analysis |
| Procedure | From the sensor gain/zero, divider and ADC reference compute the measurable current range at the ADC, the resolution, and the expected `CUR_A_PER_COUNT` and `CUR_ZERO_COUNTS`; compare with the firmware values. |
| Pass criteria | Range ≥ ±1.2 × `INV_I_TRIP_A` at the ADC and at the sensor; ≤ 1 A/count; firmware scaling within 0.5 % and zero within 1 count. |

### HVC-AN-002 Phase current filter and ADC loading

| | |
|---|---|
| Verifies | HWR-SNS-001 |
| Method | Analysis |
| Procedure | Thevenin resistance of the divider with the filter capacitor: corner frequency, phase lag at `CURRENT_LOOP_BW_HZ`, ratio of filter capacitance to ADC sampling capacitance. |
| Pass criteria | Corner ≥ 5 kHz; phase lag ≤ 5°; capacitance ratio ≥ 200. |

### HVC-AN-003 Phase current offset and gain error budget

| | |
|---|---|
| Verifies | HWR-SNS-002, HWR-PWR-004 |
| Method | Analysis |
| Procedure | Worst-case zero offset = sensor offset + ratiometric shift from the 5 V supply tolerance; for the precision supply (design) and a standard ±2 % regulator (comparison). Gain error = sensor + supply + reference tolerance. |
| Pass criteria | Offset ≤ 60 % of `INV_I_OFFSET_MAX_A` with the design supply; gain error ≤ 3 %. |

### HVC-AN-004 DC link voltage measurement

| | |
|---|---|
| Verifies | HWR-SNS-003 |
| Method | Analysis |
| Procedure | Divider ratio × isolated amplifier gain × difference amplifier gain: full scale, isolated amplifier input range, resolution; resistor voltage and power stress at 1.15 × OV trip; compare with the firmware scaling and component defines. |
| Pass criteria | Full scale and amplifier range ≥ 1.15 × `HV_OV_TRIP_V`; ≤ 0.5 V/count; firmware within 0.5 %; resistor stress ≤ 50 % of ratings. |

### HVC-AN-005 Temperature measurement

| | |
|---|---|
| Verifies | HWR-SNS-004 |
| Method | Analysis |
| Procedure | Beta model in the pull-up divider: °C per count at both trip temperatures; voltage at −40 °C and 175 °C nominal and with 1 % R25 / pull-up tolerance, against the 50 mV open/short thresholds. |
| Pass criteria | ≤ 0.5 °C/count; valid range strictly inside both thresholds. |

### HVC-AN-006 Accelerator pedal inputs

| | |
|---|---|
| Verifies | HWR-IF-001 |
| Method | Analysis |
| Procedure | Out-of-range thresholds and highest valid signal at the ADC; sensor supply at +2 % against the high threshold; clamp injection current with the signal at 16 V; filter corner; firmware scaling. |
| Pass criteria | Thresholds within 3…97 % of full scale; supply short detected; injection ≤ 50 % of the pin limit; corner 50 Hz…1 kHz; firmware within 0.5 %. |

### HVC-AN-007 Digital switch inputs

| | |
|---|---|
| Verifies | HWR-IF-002 |
| Method | Analysis |
| Procedure | Pin voltage at 8 V and 2 V input against VIH/VIL; wetting current at 9 V; resistor power at 16 V and 24 V; clamp current at 24 V and −14 V; filter time constant. |
| Pass criteria | HIGH/LOW recognised; wetting ≥ 2 mA; resistor ≤ 50 % rating continuous, ≤ 100 % during jump start; reverse current ≤ 1 mA; τ 20 µs…1 ms. |

### HVC-AN-008 Precharge circuit

| | |
|---|---|
| Verifies | HWR-HV-002 |
| Method | Analysis |
| Procedure | RC charging with maximum R and C to the firmware ratio; peak current at maximum battery voltage and minimum R; resistor energy per precharge and for 5 consecutive precharges. |
| Pass criteria | t95 ≤ `PRECHARGE_TIMEOUT_MS` / 3; peak ≤ 10 A; energy ≤ 50 % of single-pulse rating and ≤ burst rating. |

### HVC-AN-009 Active discharge

| | |
|---|---|
| Verifies | HWR-HV-003 |
| Method | Analysis |
| Procedure | RC discharge from `HV_OV_TRIP_V` to `HV_SAFE_V` with maximum C; resistor energy; firmware timeout margin. |
| Pass criteria | ≤ 1 s; `DISCHARGE_TIMEOUT_MS` ≥ 3 × discharge time; energy ≤ 50 % rating. |

### HVC-AN-010 Passive discharge

| | |
|---|---|
| Verifies | HWR-HV-004 |
| Method | Analysis |
| Procedure | RC discharge through the bleeder string only; power and voltage per resistor at `HV_OV_TRIP_V`. |
| Pass criteria | ≤ 120 s; power ≤ 50 %, voltage ≤ 75 % of ratings. |

### HVC-AN-011 DC link capacitor rating

| | |
|---|---|
| Verifies | HWR-HV-005 |
| Method | Analysis |
| Procedure | Compare the capacitor voltage rating with the overvoltage trip. |
| Pass criteria | Rating ≥ 1.05 × `HV_OV_TRIP_V`. |

### HVC-AN-012 Power module ratings

| | |
|---|---|
| Verifies | HWR-PS-001 |
| Method | Analysis |
| Procedure | Compare module blocking voltage and nominal current with the firmware trip levels. |
| Pass criteria | V_CES ≥ 1.6 × `HV_OV_TRIP_V`; I_C ≥ 1.4 × `INV_I_TRIP_A`. |

### HVC-AN-013 PWM, dead time and sampling timing

| | |
|---|---|
| Verifies | HWR-PS-002, HWR-SNS-006 |
| Method | Analysis |
| Procedure | PWM period and dead time in motor-control clock counts; dead time against switch turn-off plus driver skew; time from the last switching edge to the current sample at `INV_DUTY_MAX`. |
| Pass criteria | Integer counts within register ranges; dead time ≥ required; ≥ 1 µs before sampling. |

### HVC-AN-014 CAN bit timing and oscillator tolerance

| | |
|---|---|
| Verifies | HWR-IF-005, HWR-MCU-001 |
| Method | Analysis |
| Procedure | Read PRESDIV, PROPSEG, PSEG1, PSEG2, RJW from `bsp_can.c`; compute bit rate, sample point and the maximum oscillator tolerance from the two ISO 11898-1 conditions. |
| Pass criteria | 500 kbit/s; sample point 85…90 %; SJW ≤ PSEG2; tolerance ≥ crystal tolerance. |

### HVC-AN-015 Low-voltage power budget

| | |
|---|---|
| Verifies | HWR-PWR-005, HWR-GD-001 |
| Method | Analysis |
| Procedure | Sum the load currents per regulator; 3.3 V LDO dissipation and junction temperature at 105 °C; gate drive power Qg·ΔV·f per switch; 5 V gate-drive buck load for six channels. |
| Pass criteria | Every regulator ≤ 70 % loaded; LDO Tj ≤ Tj,max − 10 K; gate power ≤ 70 % of the isolated supply rating. |

## 3. Reviews

### HVC-RV-001 Schematic review against data sheets

| | |
|---|---|
| Verifies | HWR-PWR-002, HWR-PWR-003, HWR-MCU-001, HWR-MCU-002, HWR-MCU-003, HWR-GD-001, HWR-GD-002, HWR-GD-003, HWR-GD-004, HWR-SNS-005, HWR-IF-003, HWR-IF-004 |
| Method | Review |
| Procedure | For every IC: supply ranges, absolute maximum ratings, pin functions, required external components and application circuit (MCU supply/ballast, reset, JTAG; gate driver DESAT/clamp/FLT; AD2S1210 strapping and excitation; CAN transceiver; low-side switch clamp energy against coil energy). Replace every *assumed* value in `tools/hw_design.py` with the data sheet value and rerun `make hw-report`. |
| Pass criteria | All findings closed; no remaining assumed values; hw-report without PASS\*. |

### HVC-RV-002 Layout review: HV/LV separation, creepage and clearance

| | |
|---|---|
| Verifies | HWR-ISO-001 |
| Method | Review |
| Procedure | Check the PCB layout against IEC 60664-1 for the maximum working voltage, pollution degree and material group: reinforced creepage/clearance across every isolation barrier (gate drivers, AMC1311, transformers), slots where needed, no copper crossing the barrier. |
| Pass criteria | All distances ≥ required; review record signed. |

### HVC-RV-003 Component qualification review

| | |
|---|---|
| Verifies | HWR-ENV-001 |
| Method | Review |
| Procedure | Check every BOM line for AEC-Q100/Q101/Q200 qualification and the −40…+105 °C ambient (plus self-heating) temperature grade. |
| Pass criteria | All parts qualified, or a documented justification. |

## 4. Bench tests

### HVC-BN-001 Low-voltage power-up, supply rails and supply transients

| | |
|---|---|
| Verifies | HWR-PWR-001, HWR-PWR-002, HWR-PWR-003, HWR-PWR-004, HWR-MCU-003 |
| Method | Bench test |
| Procedure | 1. Power up at 9, 13.5, 16 V; measure all rails, ripple, start-up sequence and RESET_B timing. 2. 24 V for 60 s, −14 V for 60 s, ISO 16750-2 pulses (with generator). 3. Short each pedal sensor supply to GND and to 16 V. 4. Measure 3V3_A and 5V_PREC accuracy at −40, 25, 105 °C. |
| Pass criteria | Rails within tolerance; reset held until 3V3 valid; no damage, normal operation after each transient; sensor supplies survive shorts and recover; 3V3_A and 5V_PREC within ±0.5 %. |

### HVC-BN-002 Clock and debug interface

| | |
|---|---|
| Verifies | HWR-MCU-001, HWR-MCU-002 |
| Method | Bench test |
| Procedure | Connect the debugger via JTAG, program flash, run; measure the crystal frequency via CLKOUT with a frequency counter. |
| Pass criteria | Programming and debugging work; frequency within ±50 ppm at 25 °C. |

### HVC-BN-003 Analog front-end accuracy

| | |
|---|---|
| Verifies | HWR-SNS-001, HWR-SNS-002, HWR-SNS-003, HWR-SNS-004 |
| Method | Bench test |
| Procedure | 1. Pass known DC currents (−600…+600 A, or multi-turn primary) through each current sensor; read ADC counts. 2. Apply known DC link voltages (0…450 V) to the sense string. 3. Replace NTCs with precision resistors for −40…175 °C. Read values in the debugger. |
| Pass criteria | Current within ±2 % + offset limit of HWR-SNS-002; voltage within ±1 %; temperature within ±2 °C; open/short substitutes detected. |

### HVC-BN-004 Resolver interface

| | |
|---|---|
| Verifies | HWR-SNS-005 |
| Method | Bench test |
| Procedure | Connect the resolver; check excitation amplitude and frequency; rotate the shaft and log the angle; disconnect each winding in turn. |
| Pass criteria | Excitation per RDC data sheet; monotonic angle, no missing codes at 16 bit; fault output within 2 ms of each disconnection. |

### HVC-BN-005 Pedal and digital input thresholds

| | |
|---|---|
| Verifies | HWR-IF-001, HWR-IF-002 |
| Method | Bench test |
| Procedure | Sweep each switch input from 0 to 16 V and back; measure the switching thresholds and wetting current; apply 24 V and −14 V. Sweep each pedal input over 0…5 V and short it to 16 V. |
| Pass criteria | Switching between 2 V and 8 V; wetting ≥ 2 mA at 9 V; no damage; pedal counts match the analysis within ±1 %; injection current ≤ 1 mA. |

### HVC-BN-006 Low-side outputs with contactor loads

| | |
|---|---|
| Verifies | HWR-IF-003, HWR-HV-001 |
| Method | Bench test |
| Procedure | Drive real contactor and relay coils; measure the drain clamp voltage and the release time; short each output to battery; hold the MCU in reset and remove its supply. |
| Pass criteria | Clamp within the switch rating; release time per contactor data sheet; switch protects itself and reports the fault; all coils off in reset and unpowered. |

### HVC-BN-007 CAN physical layer

| | |
|---|---|
| Verifies | HWR-IF-004 |
| Method | Bench test |
| Procedure | Bus with a CAN analyser at 500 kbit/s, 40 m cable, other nodes; measure differential levels, bit timing and error counters under bus load; ground offset ±2 V. |
| Pass criteria | No error frames; levels per ISO 11898-2; transmit/receive work with ground offset. |

### HVC-BN-008 Gate drive waveforms and double-pulse test

| | |
|---|---|
| Verifies | HWR-GD-001, HWR-GD-004, HWR-PS-002 |
| Method | Bench test |
| Procedure | Double-pulse test per switch at 400 V with an inductive load, 100…800 A: measure gate voltage, V_CE overshoot, di/dt, turn-on/off delays and losses; check gate voltage stays below threshold on the complementary switch during the opposite switch's turn-on (Miller clamp). Measure isolated supply voltages under 10 kHz switching. |
| Pass criteria | +15/−5 V ±5 %; V_CE overshoot within module rating; no parasitic turn-on; measured turn-off time + skew < dead time. |

### HVC-BN-009 Desaturation short-circuit test

| | |
|---|---|
| Verifies | HWR-GD-002, HWR-GD-003 |
| Method | Bench test |
| Procedure | Hard short circuit and short through the load at 400 V (with protective setup); measure the time from short to soft turn-off, the nFAULT assertion, and the FlexPWM outputs. Pull GATE_EN low with PWM active. |
| Pass criteria | Turn-off within the module's short-circuit withstand time; nFAULT asserted; all six PWM outputs off by hardware; GATE_EN low disables all drivers. |

### HVC-BN-010 High-voltage precharge and discharge

| | |
|---|---|
| Verifies | HWR-HV-001, HWR-HV-002, HWR-HV-003, HWR-HV-004 |
| Method | Bench test |
| Procedure | With an HV supply as the battery: run the precharge sequence and record DC link voltage and current; activate the discharge from 430 V; disconnect all supplies at 430 V and record the passive discharge. Run 5 precharges in a row and measure resistor temperature. |
| Pass criteria | Measured times and peak current within the HVC-AN-008…010 results ± 20 %; resistor temperature within its rating. |

## 5. Environmental tests

### HVC-EN-001 Operating temperature and thermal cycling

| | |
|---|---|
| Verifies | HWR-ENV-001 |
| Method | Environmental test |
| Procedure | Function test at −40, 25 and 105 °C; thermal cycling per ISO 16750-4. |
| Pass criteria | Full function at all temperatures; no degradation after cycling. |

### HVC-EN-002 EMC emissions and immunity

| | |
|---|---|
| Verifies | HWR-ENV-002 |
| Method | Environmental test |
| Procedure | CISPR 25 conducted and radiated emissions with the inverter switching; ISO 11452-2 / -4 immunity. |
| Pass criteria | Within agreed limits; no function deviation during immunity tests. |

### HVC-EN-003 Electrostatic discharge

| | |
|---|---|
| Verifies | HWR-ENV-003 |
| Method | Environmental test |
| Procedure | ISO 10605 contact discharge ±8 kV on all connector pins, powered and unpowered. |
| Pass criteria | No damage; functional class per test plan. |

### HVC-EN-004 Vibration and mechanical shock

| | |
|---|---|
| Verifies | HWR-ENV-004 |
| Method | Environmental test |
| Procedure | ISO 16750-3 profile for the mounting location. |
| Pass criteria | No damage, no intermittent contacts, full function afterwards. |

### HVC-EN-005 Dielectric withstand and insulation resistance

| | |
|---|---|
| Verifies | HWR-ISO-001 |
| Method | Environmental test |
| Procedure | Dielectric withstand and insulation resistance between HV and LV/chassis per ISO 6469-3, before and after environmental tests. |
| Pass criteria | No breakdown; insulation resistance above the required value. |

## 6. Traceability matrix

| Requirement | Verified by |
|---|---|
| HWR-PWR-001 | HVC-BN-001 |
| HWR-PWR-002 | HVC-RV-001, HVC-BN-001 |
| HWR-PWR-003 | HVC-RV-001, HVC-BN-001 |
| HWR-PWR-004 | HVC-AN-003, HVC-BN-001 |
| HWR-PWR-005 | HVC-AN-015 |
| HWR-MCU-001 | HVC-AN-014, HVC-RV-001, HVC-BN-002 |
| HWR-MCU-002 | HVC-RV-001, HVC-BN-002 |
| HWR-MCU-003 | HVC-RV-001, HVC-BN-001 |
| HWR-HV-001 | HVC-BN-006, HVC-BN-010 |
| HWR-HV-002 | HVC-AN-008, HVC-BN-010 |
| HWR-HV-003 | HVC-AN-009, HVC-BN-010 |
| HWR-HV-004 | HVC-AN-010, HVC-BN-010 |
| HWR-HV-005 | HVC-AN-011 |
| HWR-PS-001 | HVC-AN-012 |
| HWR-PS-002 | HVC-AN-013, HVC-BN-008 |
| HWR-GD-001 | HVC-AN-015, HVC-RV-001, HVC-BN-008 |
| HWR-GD-002 | HVC-RV-001, HVC-BN-009 |
| HWR-GD-003 | HVC-RV-001, HVC-BN-009 |
| HWR-GD-004 | HVC-RV-001, HVC-BN-008 |
| HWR-SNS-001 | HVC-AN-001, HVC-AN-002, HVC-BN-003 |
| HWR-SNS-002 | HVC-AN-003, HVC-BN-003 |
| HWR-SNS-003 | HVC-AN-004, HVC-BN-003 |
| HWR-SNS-004 | HVC-AN-005, HVC-BN-003 |
| HWR-SNS-005 | HVC-RV-001, HVC-BN-004 |
| HWR-SNS-006 | HVC-AN-013 |
| HWR-IF-001 | HVC-AN-006, HVC-BN-005 |
| HWR-IF-002 | HVC-AN-007, HVC-BN-005 |
| HWR-IF-003 | HVC-RV-001, HVC-BN-006 |
| HWR-IF-004 | HVC-RV-001, HVC-BN-007 |
| HWR-IF-005 | HVC-AN-014 |
| HWR-ENV-001 | HVC-RV-003, HVC-EN-001 |
| HWR-ENV-002 | HVC-EN-002 |
| HWR-ENV-003 | HVC-EN-003 |
| HWR-ENV-004 | HVC-EN-004 |
| HWR-ISO-001 | HVC-RV-002, HVC-EN-005 |
