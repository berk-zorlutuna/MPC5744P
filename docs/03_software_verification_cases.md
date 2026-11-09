# Software Verification Cases

| Item | Value |
|---|---|
| Document | SVCS, verification cases for the EV drive software |
| Status | Draft, for review |
| Verifies | [02 Software Requirements](02_software_requirements.md) |
| Results | [04 Software Verification Results](04_software_verification_results.md) |

## 1. Verification strategy

| Method | Where | Description |
|---|---|---|
| Unit test | Host PC | One module in isolation (`tests/test_control.c`, `tests/test_vehicle.c`). |
| Simulation | Host PC | Closed loop: FOC software against an IPMSM model with an averaged inverter and the real one-period PWM delay (`tests/sim_motor.c`). |
| Integration test | Host PC | Complete application (`app.c` with scheduler, current loop, diagnostics, state machine, CAN) on a BSP stub, with simulated speed, voltages, BMS traffic and fault injection (`tests/harness.c`, `tests/test_integration.c`). |
| Analysis | Host PC | Automated check of build or source properties. |
| Review | Manual | Code review or static analysis tool; not automated. |
| Bench test | MPC5744P hardware | Needs the target and lab equipment; procedure given here. |

All automated cases are run with `make test`; `make report` regenerates the
results document. Each case below names the function implementing it.
`make trace` checks that every software requirement is covered by at least one
case and that this document and the test registry (`tests/vtest.c`) agree.

**Common setup for integration tests** (`h_reset()` unless stated otherwise):
pedal released, both NTCs at 25 °C, gear selector P, key off, DC link 0 V,
motor at standstill, BMS sending BMS_STATUS (ready) and BMS_LIMITS (400 A
discharge, 200 A charge) every 10 ms, current loop 10 calls per 1 ms task.
`h_reach_drive()`: 150 ms calibration, key on, DC link set to 345 V,
brake + D, brake released → DRIVE.

**Simulation conditions:** Vdc = 350 V, motor parameters from
`ev_config.h`, current loop sampling 10 kHz.

## 2. General

### SVC-GEN-001 Host build with all warnings as errors

| | |
|---|---|
| Verifies | SWR-GEN-001 |
| Method | Analysis |
| Implementation | `tests/vtest.c: svc_gen_001`, `Makefile` |
| Procedure | Build all portable sources and tests with `-std=c99 -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion -Wdouble-promotion -Werror`. |
| Pass criteria | Build succeeds (the test binary exists and runs). |

### SVC-GEN-002 Hardware access confined to the BSP

| | |
|---|---|
| Verifies | SWR-GEN-002 |
| Method | Analysis |
| Implementation | `tests/vtest.c: svc_gen_002` |
| Procedure | Scan all portable source and header files (`src/control`, `src/app`, `src/safety`, `src/comm`, `src/bsp/bsp.h`) for an `#include` of `MPC5744P.h`. |
| Pass criteria | All files found and none includes the device header. |

### SVC-REV-001 Code review: sampling instant and 1.5-sample delay compensation

| | |
|---|---|
| Verifies | SWR-CTL-009 |
| Method | Review |
| Procedure | Review `foc.c` (output angle), `bsp_pwm.c` (reload point, LDOK timing) and `bsp_adc.c` (CTU trigger) against the timing diagram: sample at reload k, duty written in period k, loaded at reload k+1, applied k+1…k+2. |
| Pass criteria | Angle advance equals 1.5·ω·Ts; duty update always happens before the next reload; findings recorded and closed. |

### SVC-REV-002 MISRA C:2012 static analysis

| | |
|---|---|
| Verifies | SWR-GEN-003 |
| Method | Review |
| Procedure | Run a MISRA C:2012 checker (e.g. PC-lint Plus, Polyspace, cppcheck premium) on `src/`. |
| Pass criteria | No mandatory/required violations without a documented deviation. |

## 3. Motor control

### SVC-CTL-001 Clarke/Park transforms

| | |
|---|---|
| Verifies | SWR-CTL-001 |
| Method | Unit test |
| Implementation | `tests/test_control.c: svc_ctl_001` |
| Procedure | 1. Transform an unbalanced current set abc → αβ → abc and αβ → dq → αβ. 2. Transform a balanced 100 A set aligned with θ = 1.1 rad. 3. Check angle wrapping helpers. |
| Pass criteria | Round-trip error < 1e-4; balanced set gives d = 100 A ± 1e-3, q = 0 ± 1e-3. |

### SVC-CTL-002 Space vector modulation

| | |
|---|---|
| Verifies | SWR-CTL-002 |
| Method | Unit test |
| Implementation | `tests/test_control.c: svc_ctl_002` |
| Procedure | 1. Zero voltage. 2. Voltage vector of 95 % of Vdc/√3 at 72 angles. 3. 150 % over-modulation request. |
| Pass criteria | 1: all duties 0.5. 2: duties in [0.02, 0.98]; reconstructed line-to-line voltages within 0.05 V of request. 3: duties stay within limits. |

### SVC-CTL-003 PI controller anti-windup

| | |
|---|---|
| Verifies | SWR-CTL-004 |
| Method | Unit test |
| Implementation | `tests/test_control.c: svc_ctl_003` |
| Procedure | Apply a constant positive error for 10 s with output limit ±5, then reverse the error. |
| Pass criteria | Integrator ≤ output limit; output leaves the positive limit on the first step after reversal. |

### SVC-CTL-004 MTPA current split

| | |
|---|---|
| Verifies | SWR-CTL-003 |
| Method | Unit test |
| Implementation | `tests/test_control.c: svc_ctl_004` |
| Procedure | For 1, 50, 150, 230, 300 and −100 Nm compute (id, iq); compute torque from the machine equation; compare current magnitude with id ± 2 A (iq re-solved for the same torque). Repeat with Ld = Lq. |
| Pass criteria | id < 0; torque error < 0.5 %; neighbouring points need more current; Ld = Lq gives id = 0. |

### SVC-CTL-005 Torque production and current loop dynamics

| | |
|---|---|
| Verifies | SWR-CTL-003, SWR-CTL-004, SWR-CTL-005 |
| Method | Simulation |
| Implementation | `tests/test_control.c: svc_ctl_005` |
| Procedure | 1. 2000 rpm, torque step 0 → 150 Nm, 50 ms. 2. 3000 rpm, step to `MOTOR_MAX_TORQUE_NM`, 50 ms. |
| Pass criteria | 1: torque within 2 % at 10 ms and within 1 % at 50 ms; id, iq within 3 A of reference; no field weakening; peak current ≤ 455 A. 2: torque within 1 % of maximum; \|i\| ≤ 455 A; peak < 550 A. |

### SVC-CTL-006 Regenerative torque at low speed

| | |
|---|---|
| Verifies | SWR-CTL-003, SWR-CTL-004 |
| Method | Simulation |
| Implementation | `tests/test_control.c: svc_ctl_006` |
| Procedure | 3000 rpm, step to −100 Nm, 50 ms. |
| Pass criteria | Torque −100 ± 1 Nm; peak current < 550 A. |

### SVC-CTL-007 Field weakening up to maximum speed

| | |
|---|---|
| Verifies | SWR-CTL-006, SWR-CTL-009 |
| Method | Simulation |
| Implementation | `tests/test_control.c: svc_ctl_007` |
| Procedure | 40 Nm request; speed ramp 0 → 12000 rpm in 1.5 s, then hold 0.5 s. |
| Pass criteria | At the end: FW current < −10 A; modulation index ≤ 0.97; \|i\| ≤ 455 A; peak < 550 A; id, iq within 5 A of reference; torque 40 ± 1 Nm. |

### SVC-CTL-008 Voltage-limited regeneration at maximum speed

| | |
|---|---|
| Verifies | SWR-CTL-006, SWR-CTL-007 |
| Method | Simulation |
| Implementation | `tests/test_control.c: svc_ctl_008` |
| Procedure | −60 Nm request; speed ramp 6000 → 12000 rpm in 0.5 s, hold 0.5 s. |
| Pass criteria | As SVC-CTL-007 with torque −60 ± 1.5 Nm. |

### SVC-CTL-009 Over-demand beyond voltage-limited capability

| | |
|---|---|
| Verifies | SWR-CTL-005, SWR-CTL-007 |
| Method | Simulation |
| Implementation | `tests/test_control.c: svc_ctl_009` |
| Procedure | 120 Nm request (not feasible at top speed); speed ramp 6000 → 12000 rpm, hold. |
| Pass criteria | \|i\| ≤ 455 A; peak < 550 A; modulation index ≤ 0.97; 0 < torque ≤ 120 Nm. |

### SVC-CTL-010 Inverter enable on a spinning machine

| | |
|---|---|
| Verifies | SWR-CTL-008, SWR-CTL-009 |
| Method | Simulation |
| Implementation | `tests/test_control.c: svc_ctl_010` |
| Procedure | Machine starts in ASC steady state; regulators initialised with `foc_flying_start`. 1. 9000 rpm, torque ramp to 60 Nm (1500 Nm/s). 2. 12000 rpm, 40 Nm step. 3. 6000 rpm, ramp to 150 Nm. |
| Pass criteria | Peak current < 550 A in all runs; 1 and 2 as SVC-CTL-007 criteria with 60 ± 1 Nm and 40 ± 1 Nm; 3: 150 ± 1.5 Nm. |

### SVC-CTL-011 Speed estimation

| | |
|---|---|
| Verifies | SWR-CTL-010 |
| Method | Unit test |
| Implementation | `tests/test_control.c: svc_ctl_011` |
| Procedure | 1. Constant 6000 rpm for 0.2 s. 2. Acceleration 0 → 12000 rpm in 2 s. |
| Pass criteria | 1: speed error < 0.1 %. 2: tracking error < 1 % after the first 0.1 s. |

## 4. Accelerator pedal

### SVC-PED-001 Pedal normal operation

| | |
|---|---|
| Verifies | SWR-PED-001 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_ped_001` |
| Procedure | Apply consistent track voltages for 0 %, 2.7 %, 50 %, 100 %, then 60 % / 55 %. |
| Pass criteria | 0 at 0 % and inside the dead band; dead-band-scaled value at 50 % and 1.0 at 100 % (± 0.005); the lower track (55 %) is used. |

### SVC-PED-002 Pedal range check

| | |
|---|---|
| Verifies | SWR-PED-002 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_ped_002` |
| Procedure | 1. Track 1 at 4.9 V for 19 ms, then 1 ms more. 2. Return to normal. 3. Both tracks at 0.1 V for 20 ms. 4. 15 ms glitch on track 1. |
| Pass criteria | 1: demand 0 immediately, fault only after 20 ms, for track 1 only. 2: fault stays latched. 3: both latched, pedal invalid. 4: no fault. |

### SVC-PED-003 Pedal track plausibility

| | |
|---|---|
| Verifies | SWR-PED-003 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_ped_003` |
| Procedure | 1. 20 % mismatch for 1 ms, 99 ms, 100 ms. 2. Tracks agree again. 3. Fresh: 8 % mismatch for 200 ms. |
| Pass criteria | 1: demand 0 from the first sample; fault latched at 100 ms, not before. 2: still invalid. 3: no fault, valid. |

### SVC-PED-004 Pedal single-track degraded mode

| | |
|---|---|
| Verifies | SWR-PED-004 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_ped_004` |
| Procedure | 1. Track 2 at 0 V (short to ground), track 1 at 50 % then 80 %. 2. Track 1 at 5 V (open), track 2 at 30 %. 3. Plausibility fault (60 % / 30 %). |
| Pass criteria | 1, 2: range fault on the failed track only, degraded mode, demand follows the healthy track (± 0.005). 3: invalid, not degraded. |

### SVC-PED-005 Pedal faults in the running application

| | |
|---|---|
| Verifies | SWR-PED-004, SWR-PED-005, SWR-TQ-011 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_ped_005` |
| Procedure | 1. DRIVE, full pedal with track 2 shorted to ground, 400 ms. 2. DRIVE, tracks at 80 % / 40 % for 300 ms; then 3000 rpm with brake, 300 ms. |
| Pass criteria | 1: APPS_RANGE set, reaction LIMP, still in DRIVE, torque command 0.3 × 230 Nm ± 1 Nm. 2: APPS_PLAUS set, torque command 0; with brake at 3000 rpm the command is negative (regen still available). |

## 5. Torque management

Unit tests in `tests/test_vehicle.c`, inputs: D, drive enabled, Vdc 350 V,
I_dis,max 400 A, I_chg,max 200 A, temperatures 40 °C, results after 2 s of
settling unless stated.

### SVC-TQ-001 Drive torque envelope

| | |
|---|---|
| Verifies | SWR-TQ-001 |
| Method | Unit test |
| Implementation | `svc_tq_001` |
| Procedure | Pedal 100 % and 50 % at 1000 rpm; pedal 100 % at 8000 rpm. |
| Pass criteria | 230 Nm, 115 Nm (± 0.01); 100 kW / ω at 8000 rpm (± 0.1 Nm). |

### SVC-TQ-002 Battery discharge current limit

| | |
|---|---|
| Verifies | SWR-TQ-002 |
| Method | Unit test |
| Implementation | `svc_tq_002` |
| Procedure | Full pedal at 3000 rpm with I_dis,max = 50 A, then 0 A. |
| Pass criteria | 350 × 50 × 0.9 / ω ± 0.1 Nm; 0 Nm with 0 A. |

### SVC-TQ-003 Regenerative braking levels and limits

| | |
|---|---|
| Verifies | SWR-TQ-003 |
| Method | Unit test |
| Implementation | `svc_tq_003` |
| Procedure | Pedal released at 2000 rpm; brake pressed; brake at 9000 rpm; brake at 3000 rpm with I_chg,max = 20 A and 0 A. |
| Pass criteria | −30 Nm; −100 Nm; −60 kW / ω; −350 × 20 / (0.9 × ω); 0 Nm. |

### SVC-TQ-004 Regeneration fade near standstill

| | |
|---|---|
| Verifies | SWR-TQ-004 |
| Method | Unit test |
| Implementation | `svc_tq_004` |
| Procedure | Brake pressed at 801, 500, 200 and −300 rpm (D). |
| Pass criteria | −100 Nm; −75 Nm (50 % of 150 Nm); 0; 0. |

### SVC-TQ-005 Brake-throttle override

| | |
|---|---|
| Verifies | SWR-TQ-005 |
| Method | Unit test |
| Implementation | `svc_tq_005` |
| Procedure | 3000 rpm: brake + 20 % pedal; brake + 100 %; brake released at 100 %; pedal 6 %; pedal 4 %; pedal 50 %. |
| Pass criteria | No override at 20 %; override at 100 % with negative torque; override held at 100 % and 6 %; cleared at 4 %; positive torque at 50 %. |

### SVC-TQ-006 Reverse gear torque and speed limit

| | |
|---|---|
| Verifies | SWR-TQ-006 |
| Method | Unit test |
| Implementation | `svc_tq_006` |
| Procedure | R, full pedal at −500, −2500 and −2350 rpm. |
| Pass criteria | −92 Nm; 0; −46 Nm ± 0.5. |

### SVC-TQ-007 Forward speed limit

| | |
|---|---|
| Verifies | SWR-TQ-007 |
| Method | Unit test |
| Implementation | `svc_tq_007` |
| Procedure | Full pedal at 12000, 11850 and 11699 rpm. |
| Pass criteria | ≤ 0; 50 % of the power-limited torque; 100 % (± 0.5 Nm). |

### SVC-TQ-008 Thermal and undervoltage derating

| | |
|---|---|
| Verifies | SWR-TQ-008 |
| Method | Unit test |
| Implementation | `svc_tq_008` |
| Procedure | Full pedal at 1000 rpm with T_inv = 100, 112.5, 125 °C; T_mot = 152.5 °C; Vdc = 240 V. |
| Pass criteria | 100 %, 50 %, 0 %, 50 %, 50 % of 230 Nm (± 0.5 Nm). |

### SVC-TQ-009 Torque slew rate

| | |
|---|---|
| Verifies | SWR-TQ-009 |
| Method | Unit test |
| Implementation | `svc_tq_009` |
| Procedure | Step to full pedal at 1000 rpm; record the slope for 400 ms. |
| Pass criteria | Slope ≤ 400 Nm/s while \|T\| < 20 Nm; ≤ 1500 Nm/s elsewhere and > 90 % of it (limit actually used). |

### SVC-TQ-010 Torque removal and neutral/park

| | |
|---|---|
| Verifies | SWR-TQ-010 |
| Method | Unit test |
| Implementation | `svc_tq_010` |
| Procedure | From full torque: TORQUE_OFF reaction; drive disabled; gear N; gear P. |
| Pass criteria | 0 Nm in the first step for TORQUE_OFF and drive disabled; 0 in N and P. |

### SVC-TQ-011 Limp mode scaling

| | |
|---|---|
| Verifies | SWR-TQ-011 |
| Method | Unit test |
| Implementation | `svc_tq_011` |
| Procedure | Full pedal at 1000 rpm with reaction LIMP. |
| Pass criteria | 69 Nm ± 0.1. |

## 6. Vehicle state machine

Unit tests in `tests/test_vehicle.c`, called every 10 ms; battery voltage 350 V.

### SVC-VSM-001 Standby state outputs

| | |
|---|---|
| Verifies | SWR-VSM-001 |
| Method | Unit test |
| Implementation | `svc_vsm_001` |
| Procedure | Run 50 ms after init; then key on with BMS not ready for 100 ms. |
| Pass criteria | STANDBY, all contactors and discharge off, inverter disabled, gear P; no transition without BMS ready. |

### SVC-VSM-002 Precharge sequence

| | |
|---|---|
| Verifies | SWR-VSM-002, SWR-VSM-003 |
| Method | Unit test |
| Implementation | `svc_vsm_002` |
| Procedure | 1. Key on with DC link already at 345 V; observe for 170 ms. 2. Fresh: DC link 94 % of battery for 500 ms, then 96 %. |
| Pass criteria | 1: HV− and precharge closed on entry; HV+ not before 100 ms; HV+ closed with precharge still closed; precharge open and READY 50 ms later. 2: no HV+ at 94 %; READY at 96 %. |

### SVC-VSM-003 Precharge timeout

| | |
|---|---|
| Verifies | SWR-VSM-004 |
| Method | Unit test |
| Implementation | `svc_vsm_003` |
| Procedure | Key on, DC link stays at 100 V for 3.2 s. |
| Pass criteria | Failure flagged at 3000 ± 10 ms; SHUTDOWN; HV+ and precharge open. |

### SVC-VSM-004 Conditions for entering DRIVE

| | |
|---|---|
| Verifies | SWR-VSM-005 |
| Method | Unit test |
| Implementation | `svc_vsm_004` |
| Procedure | In READY select D: without brake; with brake at 160 rpm; with inverter not ready; then all conditions met with reaction LIMP. Repeat with R. |
| Pass criteria | No transition in the first three cases; DRIVE with gear D (inverter enabled) in the fourth; DRIVE with R. |

### SVC-VSM-005 Gear changes in DRIVE

| | |
|---|---|
| Verifies | SWR-VSM-006 |
| Method | Unit test |
| Implementation | `svc_vsm_005` |
| Procedure | At 3000 rpm request R, P, N, R, D. At standstill request R without brake, R with brake, then P. |
| Pass criteria | R and P rejected at speed; N accepted (inverter stays enabled); N → R rejected, N → D accepted while rolling forward; R only with brake; P → READY with inverter disabled. |

### SVC-VSM-006 Key off while moving

| | |
|---|---|
| Verifies | SWR-VSM-007 |
| Method | Unit test |
| Implementation | `svc_vsm_006` |
| Procedure | DRIVE at 3000 rpm, key off; reduce speed to 510 rpm, then 490 rpm. |
| Pass criteria | DRIVE with N, inverter enabled, HV+ closed at 3000 and 510 rpm; SHUTDOWN at 490 rpm. |

### SVC-VSM-007 Shutdown and active discharge

| | |
|---|---|
| Verifies | SWR-VSM-008 |
| Method | Unit test |
| Implementation | `svc_vsm_007` |
| Procedure | DRIVE at standstill, key off; DC link stays 345 V for 150 ms, then 59 V. |
| Pass criteria | First step: HV+ and precharge open, HV− closed, inverter off. After 50 ms: HV− open, discharge on. Stays in SHUTDOWN while > 60 V; STANDBY with discharge off below 60 V. |

### SVC-VSM-008 Discharge timeout

| | |
|---|---|
| Verifies | SWR-VSM-009 |
| Method | Unit test |
| Implementation | `svc_vsm_008` |
| Procedure | READY, key off, DC link stays at 345 V. |
| Pass criteria | Failure flagged at most 5070 ms after key off; STANDBY with discharge off. |

### SVC-VSM-009 Fault reactions in the state machine

| | |
|---|---|
| Verifies | SWR-VSM-010 |
| Method | Unit test |
| Implementation | `svc_vsm_009` |
| Procedure | DRIVE + TORQUE_OFF; reaction removed for 100 ms; HV_OFF. READY + TORQUE_OFF. STANDBY + key on with HV_OFF. |
| Pass criteria | FAULT with inverter off and HV kept; stays FAULT; SHUTDOWN with HV+ open; READY → FAULT; no precharge with HV_OFF. |

### SVC-VSM-010 Implausible gear selector

| | |
|---|---|
| Verifies | SWR-VSM-011 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_vsm_010` |
| Procedure | READY with brake: D and R inputs both active for 100 ms; no input active for 100 ms; then D only. |
| Pass criteria | READY in both invalid cases; DRIVE with valid D. |

## 7. Diagnostics

### SVC-DIAG-001 Reaction priority and debouncing

| | |
|---|---|
| Verifies | SWR-DIAG-001 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_diag_001` |
| Procedure | Set faults of increasing severity; debounce a condition for 9 ms, interrupt, 9 ms, 10 ms, remove; check all faults for a reaction. |
| Pass criteria | Reaction = most severe; mask contains all set faults; fault only after 10 uninterrupted ms; stays latched; every fault has a reaction other than NONE. |

### SVC-DIAG-002 Fault latching and clearing

| | |
|---|---|
| Verifies | SWR-DIAG-002 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_002` |
| Procedure | DRIVE; inverter NTC at 130 °C for 250 ms; back to 25 °C for 200 ms; key off/on within 60 ms (during SHUTDOWN); key off, discharge to STANDBY; key on. |
| Pass criteria | Fault and FAULT state persist after the cause is gone and across the key cycle outside STANDBY; fault still set in STANDBY; cleared within 5 ms of the key-on edge in STANDBY. |

### SVC-DIAG-003 Hardware overcurrent input

| | |
|---|---|
| Verifies | SWR-DIAG-003 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_003` |
| Procedure | DRIVE (PWM running); activate the hardware fault input; one current loop call; then 20 ms. |
| Pass criteria | HW_OVERCURRENT set and PWM off after one call; FAULT within 20 ms with HV+ closed and torque command 0. |

### SVC-DIAG-004 Software phase overcurrent

| | |
|---|---|
| Verifies | SWR-DIAG-004 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_004` |
| Procedure | DRIVE; phase A at 535 A for one call; then phase B at −565 A for one call. |
| Pass criteria | No fault at 535 A; SW_OVERCURRENT set and PWM off after the −565 A call. |

### SVC-DIAG-005 Phase current sum plausibility

| | |
|---|---|
| Verifies | SWR-DIAG-005 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_005` |
| Procedure | DRIVE; phase A at 50 A, others 0 A (sum 50 A); 4 calls, then 1 more. |
| Pass criteria | No fault after 4; CURRENT_SUM set and PWM off after 5. |

### SVC-DIAG-006 DC link overvoltage

| | |
|---|---|
| Verifies | SWR-DIAG-006 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_006` |
| Procedure | DRIVE; 425 V for 100 calls; 435 V for 1 call. |
| Pass criteria | No fault at 425 V; HV_OVERVOLT set and PWM off after one call at 435 V. |

### SVC-DIAG-007 DC link undervoltage

| | |
|---|---|
| Verifies | SWR-DIAG-007 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_007` |
| Procedure | STANDBY at 0 V for 200 ms. DRIVE; 210 V for 49 ms, 1 ms, 10 ms. |
| Pass criteria | No fault in STANDBY; no fault after 49 ms; fault after 50 ms; FAULT state 10 ms later. |

### SVC-DIAG-008 Over-temperature and temperature measurement

| | |
|---|---|
| Verifies | SWR-DIAG-008 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_008` |
| Procedure | Inverter NTC at 130 °C for 199 ms, then 1 ms; read T_inv from the last VCU_DIAG frame. Same for the motor NTC at 170 °C. Both NTCs 3 °C below trip for 1 s. |
| Pass criteria | Fault after exactly 200 ms, not before; reported temperature 130 ± 1 °C; no fault 3 °C below trip. |

### SVC-DIAG-009 Temperature sensor open/short

| | |
|---|---|
| Verifies | SWR-DIAG-009 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_009` |
| Procedure | Inverter NTC input at full scale (open), motor NTC input at 0 (short), 205 ms. |
| Pass criteria | INV_OVERTEMP and MOT_OVERTEMP set. |

### SVC-DIAG-010 Rotor position sensor fault

| | |
|---|---|
| Verifies | SWR-DIAG-010 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_010` |
| Procedure | DRIVE; position fault indication for 19 calls, then 1 more. |
| Pass criteria | No fault after 19; POS_SENSOR set and PWM off after 20. |

### SVC-DIAG-011 Overspeed and speed measurement

| | |
|---|---|
| Verifies | SWR-DIAG-011, SWR-CTL-010 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_011` |
| Procedure | Simulated resolver at 12500 rpm for 300 ms; fresh start at 13500 rpm for 100 ms. |
| Pass criteria | Speed estimate within 1 % of 12500 rpm and no fault; OVERSPEED set at 13500 rpm. |

### SVC-DIAG-012 BMS communication loss and BMS fault

| | |
|---|---|
| Verifies | SWR-DIAG-012, SWR-COM-003 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_012` |
| Procedure | DRIVE; stop BMS frames; check at 280 ms and 320 ms; 20 ms later check state. Fresh DRIVE; BMS fault flag set for 30 ms. |
| Pass criteria | No fault at 280 ms; BMS_TIMEOUT at 320 ms; SHUTDOWN with HV+ open. BMS_FAULT set, SHUTDOWN, HV+ open. |

### SVC-DIAG-013 Precharge failure handling

| | |
|---|---|
| Verifies | SWR-DIAG-013, SWR-VSM-004 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_diag_013` |
| Procedure | Key on, DC link stuck at 100 V for 3.1 s; DC link to 0 V for 1 s with key held on; key cycle; DC link 345 V. |
| Pass criteria | PRECHARGE fault set, precharge relay and HV+ open; STANDBY without new attempt (HV− open); after key cycle a new precharge starts and reaches READY. |

## 8. Safe state and supervision

### SVC-SAFE-001 Freewheel / active short circuit selection

| | |
|---|---|
| Verifies | SWR-SAFE-001 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_safe_001` |
| Procedure | DRIVE at 6000 rpm; inject MOT_OVERTEMP; then 5000, 4000, 5000, 6000 rpm (50 ms each). |
| Pass criteria | ASC immediately at 6000; ASC at 5000 (hysteresis); off at 4000; off at 5000; ASC at 6000; state FAULT. |

### SVC-SAFE-002 Hardware overcurrent at high speed

| | |
|---|---|
| Verifies | SWR-SAFE-002 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_safe_002` |
| Procedure | DRIVE at 6500 rpm; activate the hardware fault input; 1 call, then 50 ms. |
| Pass criteria | PWM off (not ASC) throughout. |

### SVC-SAFE-003 Task supervision and watchdog service

| | |
|---|---|
| Verifies | SWR-SAFE-003 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_safe_003` |
| Procedure | 200 ms nominal; 100 ms with current loop stopped; 100 ms at half rate (5 calls/ms); 100 ms with the 1 ms task stopped; 100 ms nominal. |
| Pass criteria | ≥ 18 services in the first 200 ms; no service in any of the three fault phases; services resume. |

### SVC-SAFE-004 Safe outputs at start-up and in STANDBY

| | |
|---|---|
| Verifies | SWR-SAFE-004 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_safe_004` |
| Procedure | After `app_init` and after 300 ms in STANDBY. |
| Pass criteria | PWM disabled; all digital outputs (contactors, discharge, gate enable) off; state STANDBY. |

### SVC-SAFE-005 Current sensor offset calibration

| | |
|---|---|
| Verifies | SWR-SAFE-005, SWR-CTL-011 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_safe_005` |
| Procedure | 1. 1023 and 1024 current loop calls after reset. 2. 10 A offset on phase A. 3. 20 A offset on phase B; key on; DC link 345 V, brake + D for 600 ms. |
| Pass criteria | 1: not calibrated and PWM off after 1023; calibrated after 1024, no fault. 2: no fault. 3: CURRENT_OFFSET set, still set after the key-on edge, DRIVE never entered, PWM off. |

## 9. Communication

### SVC-COM-001 BMS message decoding

| | |
|---|---|
| Verifies | SWR-COM-001 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_com_001` |
| Procedure | Decode BMS_STATUS (355.2 V, −20.0 A, 80 %, ready) and BMS_LIMITS (400 A / 200 A); then a status frame with fault set. |
| Pass criteria | All values within one LSB; flags correct. |

### SVC-COM-002 Rejection of invalid BMS frames

| | |
|---|---|
| Verifies | SWR-COM-002 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_com_002` |
| Procedure | Repeat a frame with the same alive counter; corrupt a byte; DLC 7; foreign ID; then a valid frame. |
| Pass criteria | First four rejected with data unchanged; the valid frame accepted. |

### SVC-COM-003 BMS message timeout

| | |
|---|---|
| Verifies | SWR-COM-003 |
| Method | Unit test |
| Implementation | `tests/test_vehicle.c: svc_com_003` |
| Procedure | Check before any frame; after both frames; after 299 ms and 300 ms; with only BMS_STATUS received. |
| Pass criteria | Timed out; not timed out; not at 299 ms; timed out at 300 ms; timed out with BMS_LIMITS missing. |

### SVC-COM-004 Periodic VCU messages

| | |
|---|---|
| Verifies | SWR-COM-004 |
| Method | Integration test |
| Implementation | `tests/test_integration.c: svc_com_004` |
| Procedure | STANDBY; record all transmitted frames over 100 ms. |
| Pass criteria | 10 VCU_STATUS and 10 VCU_DIAG frames, DLC 8; state byte = STANDBY, gear byte = P; alive counter increments by 1 per frame. |

## 10. Hardware bench tests (target required)

Equipment: MPC5744P board with the inverter control board, debugger
(S32DS / PE Micro or Lauterbach), 4-channel oscilloscope, CAN interface with
analyser software, adjustable LV supply; for SVC-HW-009 a resolver and
RDC. No high voltage is needed for these cases (gate drivers may be
disconnected).

### SVC-HW-001 Clock frequencies

| | |
|---|---|
| Verifies | SWR-HW-001 |
| Method | Bench test |
| Procedure | Route the system clock and the motor control clock to CLKOUT (MC_CGM output clock divider) and measure; read MC_ME.GS for the current system clock source. |
| Pass criteria | 200 MHz and 160 MHz within crystal tolerance (± 100 ppm); PLL1 selected. |

### SVC-HW-002 PWM frequency, alignment and dead time

| | |
|---|---|
| Verifies | SWR-HW-002 |
| Method | Bench test |
| Procedure | Force duties 25/50/75 % (debugger); measure all six outputs; check outputs before `bsp_pwm_enable`. |
| Pass criteria | 10 kHz ± 0.1 %; pulses centred on the same instant; dead time 1.5 µs ± 50 ns on every edge; complementary; all low while masked. |

### SVC-HW-003 Hardware fault input shutdown time

| | |
|---|---|
| Verifies | SWR-HW-003 |
| Method | Bench test |
| Procedure | PWM running; pull the FAULT0 pin low with a pulse generator; measure from fault edge to outputs low; release the pin; try to re-enable without and with clearing. |
| Pass criteria | Outputs low within 1 µs (plus configured filter); stay off after release until software clears; flag visible to software. |

### SVC-HW-004 ADC trigger instant and simultaneous sampling

| | |
|---|---|
| Verifies | SWR-HW-004 |
| Method | Bench test |
| Procedure | Toggle a GPIO at the start of `bsp_isr_adc1_eoc`; scope the PWM and the GPIO; feed a known DC voltage to each current channel. |
| Pass criteria | One ISR per PWM period; conversion starts 500 ns after the counter reload (centre of the zero vector); measured counts match the input within ± 4 LSB. |

### SVC-HW-005 CAN bit timing and mailbox operation

| | |
|---|---|
| Verifies | SWR-HW-005 |
| Method | Bench test |
| Procedure | Connect a CAN analyser at 500 kbit/s; send BMS frames; observe VCU frames; measure bit time and sample point with a scope. |
| Pass criteria | No error frames; 0x301/0x302 every 10 ms ± 1 ms; BMS frames decoded (debugger); bit time 2 µs; sample point 87.5 %. |

### SVC-HW-006 Task rates on target

| | |
|---|---|
| Verifies | SWR-HW-006 |
| Method | Bench test |
| Procedure | Toggle one GPIO in the current loop and another in the 1 ms task; scope both. |
| Pass criteria | 10 kHz and 1 kHz; the current loop pre-empts the 1 ms task (no jitter > 2 µs on the fast GPIO). |

### SVC-HW-007 Current loop execution time

| | |
|---|---|
| Verifies | SWR-HW-007 |
| Method | Bench test |
| Procedure | GPIO high at entry and low at exit of the current loop ISR (or cycle counter); run in DRIVE with field weakening active (simulated inputs) for 10 min; record maximum. |
| Pass criteria | Maximum ≤ 50 µs. |

### SVC-HW-008 Watchdog reset on task stall

| | |
|---|---|
| Verifies | SWR-HW-008, SWR-SAFE-003 |
| Method | Bench test |
| Procedure | Debug build option that stops (a) the PIT interrupt, (b) the ADC interrupt; read the reset cause (MC_RGM) after restart; measure time from stall to reset with a GPIO. |
| Pass criteria | Reset within 50 ms + one 10 ms window in both cases; reset cause = SWT. |

### SVC-HW-009 Resolver interface and offset calibration

| | |
|---|---|
| Verifies | SWR-HW-009 |
| Method | Bench test |
| Procedure | Rotate the motor by hand / drive it externally; log the angle; disconnect the resolver; determine the electrical offset (back-EMF zero crossing vs. angle) and enter it in `RESOLVER_OFFSET_RAD`. |
| Pass criteria | Monotonic angle, 65536 counts per resolver cycle; fault reported within 20 samples after disconnection; offset determined to ± 1° electrical. |

### SVC-HW-010 Target build

| | |
|---|---|
| Verifies | SWR-HW-010 |
| Method | Bench test |
| Procedure | Import into an S32DS MPC5744P project as described in the README; build Debug and Release. |
| Pass criteria | No errors; no warnings in project sources; map file shows the current loop code and data in flash/RAM as expected. |

## 11. Traceability matrix (software requirement → cases)

| Requirement | Verified by |
|---|---|
| SWR-GEN-001 | SVC-GEN-001 |
| SWR-GEN-002 | SVC-GEN-002 |
| SWR-GEN-003 | SVC-REV-002 |
| SWR-CTL-001 | SVC-CTL-001 |
| SWR-CTL-002 | SVC-CTL-002 |
| SWR-CTL-003 | SVC-CTL-004, SVC-CTL-005, SVC-CTL-006 |
| SWR-CTL-004 | SVC-CTL-003, SVC-CTL-005, SVC-CTL-006 |
| SWR-CTL-005 | SVC-CTL-005, SVC-CTL-009 |
| SWR-CTL-006 | SVC-CTL-007, SVC-CTL-008 |
| SWR-CTL-007 | SVC-CTL-008, SVC-CTL-009 |
| SWR-CTL-008 | SVC-CTL-010 |
| SWR-CTL-009 | SVC-CTL-007, SVC-CTL-010, SVC-REV-001 |
| SWR-CTL-010 | SVC-CTL-011, SVC-DIAG-011 |
| SWR-CTL-011 | SVC-SAFE-005 |
| SWR-PED-001 | SVC-PED-001 |
| SWR-PED-002 | SVC-PED-002 |
| SWR-PED-003 | SVC-PED-003 |
| SWR-PED-004 | SVC-PED-004, SVC-PED-005 |
| SWR-PED-005 | SVC-PED-005 |
| SWR-TQ-001 | SVC-TQ-001 |
| SWR-TQ-002 | SVC-TQ-002 |
| SWR-TQ-003 | SVC-TQ-003 |
| SWR-TQ-004 | SVC-TQ-004 |
| SWR-TQ-005 | SVC-TQ-005 |
| SWR-TQ-006 | SVC-TQ-006 |
| SWR-TQ-007 | SVC-TQ-007 |
| SWR-TQ-008 | SVC-TQ-008 |
| SWR-TQ-009 | SVC-TQ-009 |
| SWR-TQ-010 | SVC-TQ-010 |
| SWR-TQ-011 | SVC-TQ-011, SVC-PED-005 |
| SWR-VSM-001 | SVC-VSM-001 |
| SWR-VSM-002 | SVC-VSM-002 |
| SWR-VSM-003 | SVC-VSM-002 |
| SWR-VSM-004 | SVC-VSM-003, SVC-DIAG-013 |
| SWR-VSM-005 | SVC-VSM-004 |
| SWR-VSM-006 | SVC-VSM-005 |
| SWR-VSM-007 | SVC-VSM-006 |
| SWR-VSM-008 | SVC-VSM-007 |
| SWR-VSM-009 | SVC-VSM-008 |
| SWR-VSM-010 | SVC-VSM-009 |
| SWR-VSM-011 | SVC-VSM-010 |
| SWR-DIAG-001 | SVC-DIAG-001 |
| SWR-DIAG-002 | SVC-DIAG-002 |
| SWR-DIAG-003 | SVC-DIAG-003 |
| SWR-DIAG-004 | SVC-DIAG-004 |
| SWR-DIAG-005 | SVC-DIAG-005 |
| SWR-DIAG-006 | SVC-DIAG-006 |
| SWR-DIAG-007 | SVC-DIAG-007 |
| SWR-DIAG-008 | SVC-DIAG-008 |
| SWR-DIAG-009 | SVC-DIAG-009 |
| SWR-DIAG-010 | SVC-DIAG-010 |
| SWR-DIAG-011 | SVC-DIAG-011 |
| SWR-DIAG-012 | SVC-DIAG-012 |
| SWR-DIAG-013 | SVC-DIAG-013 |
| SWR-SAFE-001 | SVC-SAFE-001 |
| SWR-SAFE-002 | SVC-SAFE-002 |
| SWR-SAFE-003 | SVC-SAFE-003, SVC-HW-008 |
| SWR-SAFE-004 | SVC-SAFE-004 |
| SWR-SAFE-005 | SVC-SAFE-005 |
| SWR-COM-001 | SVC-COM-001 |
| SWR-COM-002 | SVC-COM-002 |
| SWR-COM-003 | SVC-COM-003, SVC-DIAG-012 |
| SWR-COM-004 | SVC-COM-004 |
| SWR-HW-001 | SVC-HW-001 |
| SWR-HW-002 | SVC-HW-002 |
| SWR-HW-003 | SVC-HW-003 |
| SWR-HW-004 | SVC-HW-004 |
| SWR-HW-005 | SVC-HW-005 |
| SWR-HW-006 | SVC-HW-006 |
| SWR-HW-007 | SVC-HW-007 |
| SWR-HW-008 | SVC-HW-008 |
| SWR-HW-009 | SVC-HW-009 |
| SWR-HW-010 | SVC-HW-010 |
