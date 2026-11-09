# Software Requirements Specification

| Item | Value |
|---|---|
| Document | SRS, EV traction inverter and VCU software |
| Status | Draft, for review |
| Parent | [01 System Requirements](01_system_requirements.md) |
| Verification | [03 Software Verification Cases](03_software_verification_cases.md), [04 Software Verification Results](04_software_verification_results.md) |

## 1. Scope and conventions

These requirements refine the system requirements for the software running
on the MPC5744P. IDs have the form `SWR-<area>-nnn`. The **Traces to** column
gives the parent system requirements. Every software requirement is verified
by at least one case in the verification case document; `make trace` checks
this automatically.

Symbolic names in `code font` are the parameters in `config/ev_config.h` or
`config/board_cfg.h`. The values in parentheses are the current example
calibration.

Software architecture and timing (informative):

| Context | Rate | Contents |
|---|---|---|
| Current loop ISR | `PWM_FREQ_HZ` (10 kHz) | sampling, fast protections, position, FOC, PWM |
| 1 ms task | 1 kHz | inputs, CAN RX, debounced diagnostics, torque manager |
| 10 ms task (inside 1 ms task) | 100 Hz | vehicle state machine, contactors, CAN TX |
| Background | idle | task supervision, watchdog service |

## 2. General

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-GEN-001 | The software shall be written in C99 and shall compile without warnings on the host with `-Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion -Wdouble-promotion -Werror`. | SYS-064 | all |
| SWR-GEN-002 | Only the files in `src/bsp/mpc5744p/` shall access microcontroller registers. All other modules shall use the interface in `src/bsp/bsp.h`. | SYS-064 | `src/bsp/` |
| SWR-GEN-003 | The software shall comply with MISRA C:2012 (mandatory and required rules), with documented deviations. | SYS-064 | all |

## 3. Motor control (CTL)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-CTL-001 | The software shall transform phase quantities to the rotor reference frame with the amplitude-invariant Clarke and Park transforms, and back. | SYS-001 | `mc_math.h` |
| SWR-CTL-002 | The software shall convert the stationary voltage reference to three duty cycles by space vector modulation (min/max zero-sequence injection), linear up to Vdc/√3, with duty cycles limited to [`INV_DUTY_MIN`, `INV_DUTY_MAX`] (0.02 … 0.98). | SYS-001, SYS-061 | `svm.c` |
| SWR-CTL-003 | The software shall convert the torque request into d/q current references on the maximum-torque-per-ampere (MTPA) curve, with torque error below 0.5 % of the request. | SYS-001 | `foc.c: foc_mtpa` |
| SWR-CTL-004 | The software shall control the d and q currents every PWM period with PI regulators tuned for `CURRENT_LOOP_BW_HZ` (500 Hz), with back-EMF and cross-coupling feed-forward and anti-windup. After a torque step at 2000 rpm, torque shall be within 2 % after 10 ms. | SYS-001 | `foc.c: foc_step` |
| SWR-CTL-005 | The current reference magnitude shall not exceed `INV_I_MAX_A` (450 A). The maximum torque `MOTOR_MAX_TORQUE_NM` shall be achievable within this limit. | SYS-001, SYS-041 | `foc.c`, `ev_config.h` |
| SWR-CTL-006 | When the voltage demand exceeds `INV_FW_VOLTAGE_RATIO` (92 %) of the voltage limit, the software shall add negative d-axis current (up to `INV_I_FW_MAX_A`, 300 A) and recompute the q-axis current so that the delivered torque stays equal to the request (within 2.5 %) where feasible. | SYS-002 | `foc.c` |
| SWR-CTL-007 | When the request is not feasible within the voltage and current limits, the software shall limit the q-axis current to the voltage ellipse and the voltage vector magnitude to `INV_VOLTAGE_MARGIN` × Vdc/√3, so that the current stays below `INV_I_TRIP_A` (550 A) and current control is maintained. | SYS-002, SYS-042 | `foc.c` |
| SWR-CTL-008 | When the inverter is enabled on a rotating machine, the software shall initialise the regulators from the measured current and speed so that the transition causes no overcurrent trip, at any speed up to `MOTOR_MAX_RPM`. | SYS-002, SYS-042 | `foc.c: foc_flying_start`, `app.c` |
| SWR-CTL-009 | The software shall compensate the 1.5-sample delay between current sampling and the mean application of the new voltage by advancing the output angle by 1.5·ω·Ts. | SYS-002, SYS-061 | `foc.c` |
| SWR-CTL-010 | The software shall derive the electrical angle from the resolver angle, the pole pair ratio and `RESOLVER_OFFSET_RAD`, and estimate the speed with a tracking observer (`PLL_BW_HZ`, 150 Hz) with a steady-state error below 0.1 % and below 1 % during vehicle-like acceleration. | SYS-002, SYS-063 | `pll.c`, `app.c` |
| SWR-CTL-011 | At start-up, with the PWM disabled, the software shall determine the offset of each phase current sensor as the mean of 1024 samples and subtract it from all later measurements. | SYS-041 | `app.c` |

## 4. Accelerator pedal (PED)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-PED-001 | The software shall normalise both pedal tracks to 0…1 using their calibrated end points, use the lower of the two values, and apply a dead band of `APPS_DEADBAND` (3 %). | SYS-001, SYS-040 | `pedal.c` |
| SWR-PED-002 | A track voltage outside [`APPS_OOR_LOW_V`, `APPS_OOR_HIGH_V`] (0.25 … 4.75 V) shall set the pedal demand to zero immediately and shall latch a range fault for that track after `APPS_RANGE_DEBOUNCE_MS` (20 ms). | SYS-040 | `pedal.c` |
| SWR-PED-003 | While both tracks are in range, a difference of more than `APPS_PLAUS_TOL` (10 %) shall set the demand to zero immediately and latch a plausibility fault after `APPS_PLAUS_DEBOUNCE_MS` (100 ms). After a plausibility fault the demand shall stay zero until the next key cycle. | SYS-040 | `pedal.c` |
| SWR-PED-004 | If exactly one track has a latched range fault and no plausibility fault is latched, the software shall continue with the other track (degraded mode). | SYS-040 | `pedal.c` |
| SWR-PED-005 | Pedal range and plausibility faults shall be reported to the fault manager with the reaction LIMP. | SYS-040 | `app.c`, `diag.c` |

## 5. Torque management (TQ)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-TQ-001 | The drive torque request shall be pedal × available torque, where the available torque is the minimum of `MOTOR_MAX_TORQUE_NM` (230 Nm) and `MOTOR_MAX_POWER_W`/ω (100 kW). | SYS-001 | `torque_mgr.c` |
| SWR-TQ-002 | The drive torque shall not exceed Vdc × I_dis,max × η / ω, with I_dis,max from the BMS and η = `DRIVETRAIN_EFF`. With I_dis,max = 0 no drive torque shall be requested. | SYS-003 | `torque_mgr.c` |
| SWR-TQ-003 | With the pedal released the request shall be −`REGEN_COAST_NM` (30 Nm); with the brake pressed, −`REGEN_BRAKE_NM` (100 Nm). Regenerative torque shall be limited by `MOTOR_MAX_REGEN_NM` (150 Nm), `MOTOR_MAX_REGEN_POWER_W`/ω (60 kW) and Vdc × I_chg,max / (η × ω). | SYS-003, SYS-010 | `torque_mgr.c` |
| SWR-TQ-004 | The regenerative torque limit shall scale linearly from 100 % at `REGEN_FADE_HI_RPM` (800 rpm) to 0 at `REGEN_FADE_LO_RPM` (200 rpm), measured in the selected direction of travel. | SYS-011 | `torque_mgr.c` |
| SWR-TQ-005 | If the brake is pressed while the pedal is above `BTO_PEDAL_ON` (25 %), the pedal demand shall be ignored until the pedal falls below `BTO_PEDAL_OFF` (5 %). | SYS-022 | `torque_mgr.c` |
| SWR-TQ-006 | In R the torque shall be negated and scaled by `REVERSE_TORQUE_SCALE` (0.4); drive torque shall taper to zero over `SPEED_LIMIT_BAND_RPM` (300 rpm) below `REVERSE_MAX_RPM` (2500 rpm). | SYS-004 | `torque_mgr.c` |
| SWR-TQ-007 | In D the drive torque shall taper linearly to zero between `MOTOR_MAX_RPM` − 300 rpm and `MOTOR_MAX_RPM` (12000 rpm). | SYS-002 | `torque_mgr.c` |
| SWR-TQ-008 | Torque shall be scaled linearly from 100 % to 0 between `TEMP_INV_DERATE_C` and `TEMP_INV_TRIP_C` (100 … 125 °C) of the inverter, between `TEMP_MOT_DERATE_C` and `TEMP_MOT_TRIP_C` (140 … 165 °C) of the motor, and between `HV_UV_DERATE_V` and `HV_UV_TRIP_V` (260 … 220 V) of the DC link; the smallest factor applies. | SYS-045 | `torque_mgr.c` |
| SWR-TQ-009 | The torque command slope shall not exceed `TORQUE_RATE_ZERO_NMPS` (400 Nm/s) while \|T\| < `TORQUE_ZERO_BAND_NM` (20 Nm), and `TORQUE_RATE_NMPS` (1500 Nm/s) otherwise. | SYS-005 | `torque_mgr.c` |
| SWR-TQ-010 | The torque command shall become zero within one 1 ms cycle, without slew-rate limiting, on a TORQUE_OFF or HV_OFF reaction or when driving is not enabled. In P and N the command shall be zero. | SYS-020, SYS-041 | `torque_mgr.c` |
| SWR-TQ-011 | With the reaction LIMP, the torque shall be scaled by `LIMP_TORQUE_SCALE` (0.3). | SYS-040 | `torque_mgr.c` |

## 6. Vehicle state machine (VSM)

The state machine runs every 10 ms with the states INIT, STANDBY,
PRECHARGE, READY, DRIVE, FAULT and SHUTDOWN.

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-VSM-001 | In STANDBY all contactors and the discharge output shall be off, the inverter disabled and the gear P. STANDBY shall be left only when key on, BMS ready and no HV_OFF reaction. | SYS-030, SYS-047 | `vehicle_sm.c` |
| SWR-VSM-002 | On entering PRECHARGE the software shall close the HV− contactor and the precharge relay. | SYS-030 | `vehicle_sm.c` |
| SWR-VSM-003 | The HV+ contactor shall be closed when at least `PRECHARGE_MIN_MS` (100 ms) have elapsed, the battery voltage is above `HV_UV_TRIP_V`, and the DC link voltage is ≥ `PRECHARGE_RATIO` (95 %) of the battery voltage. The precharge relay shall open `CONTACTOR_DELAY_MS` (50 ms) later and the state shall become READY. | SYS-030 | `vehicle_sm.c` |
| SWR-VSM-004 | If the precharge does not complete within `PRECHARGE_TIMEOUT_MS` (3 s), the software shall report a precharge failure and enter SHUTDOWN. | SYS-030, SYS-043 | `vehicle_sm.c` |
| SWR-VSM-005 | From READY, DRIVE shall be entered only if D or R is selected, the brake is pressed, \|speed\| < `GEAR_SHIFT_MAX_RPM` (150 rpm), the inverter is ready (calibration done, no hardware fault) and the reaction is below TORQUE_OFF. | SYS-020 | `vehicle_sm.c` |
| SWR-VSM-006 | In DRIVE: N shall always be accepted; D↔R only at standstill with brake; N→D/R at standstill or when rolling in the selected direction; P only at standstill, and P shall lead to READY. | SYS-020, SYS-021 | `vehicle_sm.c` |
| SWR-VSM-007 | Key off in DRIVE at \|speed\| ≥ `KEYOFF_MAX_RPM` (500 rpm) shall select N and keep DRIVE (HV on, inverter controlled) until the speed is below that threshold. | SYS-032 | `vehicle_sm.c` |
| SWR-VSM-008 | In SHUTDOWN the software shall disable the inverter, open HV+ and the precharge relay, open HV− `CONTACTOR_DELAY_MS` later, then enable the discharge until the DC link is below `HV_SAFE_V` (60 V), and then enter STANDBY. | SYS-031 | `vehicle_sm.c` |
| SWR-VSM-009 | If the DC link is not below `HV_SAFE_V` within `DISCHARGE_TIMEOUT_MS` (5 s), the software shall report a discharge failure, switch the discharge off and enter STANDBY. | SYS-031 | `vehicle_sm.c` |
| SWR-VSM-010 | A TORQUE_OFF reaction in READY or DRIVE shall lead to FAULT (inverter disabled, HV kept) until key off. An HV_OFF reaction or key off in PRECHARGE, READY, DRIVE (subject to SWR-VSM-007) or FAULT shall lead to SHUTDOWN. | SYS-041, SYS-043 | `vehicle_sm.c` |
| SWR-VSM-011 | A gear selector state with no position or more than one position active shall be treated as invalid and shall not change the engaged gear. | SYS-023 | `app.c`, `vehicle_sm.c` |

## 7. Diagnostics (DIAG)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-DIAG-001 | Each fault shall have one of the reactions WARN, LIMP, TORQUE_OFF or HV_OFF; the most severe reaction of all active faults applies. Debounced faults shall only be set after the condition was continuously present for the debounce time. | SYS-041, SYS-044 | `diag.c` |
| SWR-DIAG-002 | Faults shall stay set until a key-on edge occurs in STANDBY; then all faults shall be cleared and re-detected if still present. | SYS-044 | `diag.c`, `app.c` |
| SWR-DIAG-003 | An active hardware overcurrent input shall set the fault HW_OVERCURRENT (TORQUE_OFF) in the next current loop cycle. | SYS-041, SYS-062 | `app.c` |
| SWR-DIAG-004 | A phase current magnitude above `INV_I_TRIP_A` (550 A) shall set SW_OVERCURRENT (TORQUE_OFF) and disable the PWM in the same current loop cycle. | SYS-041 | `app.c` |
| SWR-DIAG-005 | \|ia + ib + ic\| > `INV_I_SUM_TRIP_A` (40 A) for `INV_I_SUM_TRIP_SAMPLES` (5) consecutive samples shall set CURRENT_SUM (TORQUE_OFF). | SYS-041 | `app.c` |
| SWR-DIAG-006 | A DC link voltage above `HV_OV_TRIP_V` (430 V) shall set HV_OVERVOLT (TORQUE_OFF) in the same current loop cycle. | SYS-041 | `app.c` |
| SWR-DIAG-007 | In READY or DRIVE, a DC link voltage below `HV_UV_TRIP_V` (220 V) for 50 ms shall set HV_UNDERVOLT (TORQUE_OFF). | SYS-041 | `app.c` |
| SWR-DIAG-008 | Inverter temperature above `TEMP_INV_TRIP_C` or motor temperature above `TEMP_MOT_TRIP_C` for `TEMP_TRIP_DEBOUNCE_MS` (200 ms) shall set INV_OVERTEMP or MOT_OVERTEMP (TORQUE_OFF). Temperatures shall be computed from the NTC divider with the beta model. | SYS-041 | `app.c` |
| SWR-DIAG-009 | An NTC reading within 50 mV of 0 V or of the ADC reference (open or short circuit) shall be interpreted as 200 °C. | SYS-041 | `app.c` |
| SWR-DIAG-010 | A position sensor fault indication for `POS_SENSOR_FAULT_SAMPLES` (20) consecutive samples shall set POS_SENSOR (TORQUE_OFF). | SYS-041, SYS-063 | `app.c` |
| SWR-DIAG-011 | \|speed\| above `MOTOR_OVERSPEED_RPM` (13000 rpm) for 10 ms shall set OVERSPEED (TORQUE_OFF). | SYS-041 | `app.c` |
| SWR-DIAG-012 | With HV connected (PRECHARGE, READY, DRIVE, FAULT), a BMS message timeout shall set BMS_TIMEOUT and a BMS fault flag shall set BMS_FAULT (both HV_OFF). | SYS-043 | `app.c` |
| SWR-DIAG-013 | A precharge failure shall set PRECHARGE (HV_OFF); a new precharge attempt shall only be made after a key cycle. | SYS-043 | `app.c` |

## 8. Safe state and supervision (SAFE)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-SAFE-001 | When torque is not allowed, the current loop shall select ASC above `ASC_ENTER_RPM` (5500 rpm), keep ASC until the speed is below `ASC_EXIT_RPM` (4500 rpm), and otherwise switch all transistors off. | SYS-042 | `app.c: apply_safe_state` |
| SWR-SAFE-002 | With HW_OVERCURRENT set, all transistors shall be off at any speed (no ASC). | SYS-042 | `app.c` |
| SWR-SAFE-003 | The watchdog shall only be serviced if, in the last 10 ms window, the current loop ran 80…120 % of its nominal count and the 1 ms task ran 8…12 times. | SYS-046 | `app.c: app_background` |
| SWR-SAFE-004 | After initialisation and in STANDBY, the outputs for contactors, discharge and gate driver enable shall be off and the PWM outputs disabled. | SYS-047 | `app.c`, `bsp_io.c`, `bsp_pwm.c` |
| SWR-SAFE-005 | The PWM shall stay disabled until the current offset calibration is complete. An offset above `INV_I_OFFSET_MAX_A` (15 A) shall set CURRENT_OFFSET (TORQUE_OFF), which shall persist until the next power-up, also across key cycles. | SYS-041, SYS-047 | `app.c` |

## 9. Communication (COM)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-COM-001 | The software shall decode BMS_STATUS (0x101: pack voltage 0.1 V, pack current 0.1 A signed, SOC 0.5 %, ready/fault flags) and BMS_LIMITS (0x102: discharge and charge current limit 0.1 A), little-endian. | SYS-050 | `can_msgs.c` |
| SWR-COM-002 | A BMS frame shall be rejected if its DLC is not 8, its XOR checksum (byte 7) is wrong, or its alive counter (byte 6) did not change. | SYS-050 | `can_msgs.c` |
| SWR-COM-003 | If either BMS message has not been accepted for `BMS_TIMEOUT_MS` (300 ms), or none has been received yet, the BMS data shall be considered timed out. | SYS-043, SYS-050 | `can_msgs.c` |
| SWR-COM-004 | Every 10 ms the software shall transmit VCU_STATUS (0x301: state, gear, rpm, torque 0.1 Nm, reaction, alive counter) and VCU_DIAG (0x302: fault mask, Vdc 0.1 V, T_inv + 40, T_mot + 40). | SYS-051 | `can_msgs.c`, `app.c` |

## 10. Hardware-dependent software (HW)

| ID | Requirement | Traces to | Implemented in |
|---|---|---|---|
| SWR-HW-001 | The BSP shall configure the system clock to 200 MHz (PLL1), the motor control clock to 160 MHz and the peripheral bridge clock to 50 MHz from the 40 MHz crystal. | SYS-060 | `bsp_main.c` |
| SWR-HW-002 | FlexPWM_0 sub-modules 0–2 shall generate 10 kHz center-aligned complementary PWM with `INV_DEADTIME_NS` (1.5 µs) dead time, with outputs masked after initialisation. | SYS-061 | `bsp_pwm.c` |
| SWR-HW-003 | The FlexPWM fault input 0 shall disable all six PWM outputs in hardware and latch until cleared by software with the input inactive. | SYS-062 | `bsp_pwm.c` |
| SWR-HW-004 | The CTU shall start simultaneous conversions of the phase currents and DC link voltage on ADC_0/ADC_1 `CTU_SAMPLE_DELAY_NS` (500 ns) after each PWM reload; the current loop shall run after the last conversion. | SYS-061 | `bsp_adc.c` |
| SWR-HW-005 | FlexCAN_0 shall operate at 500 kbit/s with an 87.5 % sample point, receive 0x101/0x102 into dedicated mailboxes and transmit through two mailboxes. | SYS-050 | `bsp_can.c` |
| SWR-HW-006 | PIT_0 channel 0 shall generate the 1 ms interrupt; the current loop interrupt shall have a higher priority than the 1 ms interrupt and be able to preempt it. | SYS-046 | `bsp_main.c` |
| SWR-HW-007 | The worst-case execution time of the current loop ISR shall not exceed 50 % of the PWM period (50 µs). | SYS-046, SYS-061 | all fast-path code |
| SWR-HW-008 | The software watchdog SWT_0 shall reset the microcontroller if it is not serviced within 50 ms. | SYS-046 | `bsp_main.c` |
| SWR-HW-009 | The BSP shall latch the resolver angle and read it as a 16-bit value over DSPI, and report a fault on a converter fault signal or an SPI timeout. | SYS-063 | `bsp_io.c` |
| SWR-HW-010 | The complete software shall build for the MPC5744P (PowerPC VLE, hardware single-precision FPU) without errors in S32 Design Studio. | SYS-060 | all |
