/**
 * @file ev_config.h
 * @brief Vehicle, motor, inverter and calibration parameters.
 *
 * Every value in this file is an EXAMPLE for a ~100 kW / 350 V IPMSM drive.
 * Replace all of them with data from YOUR motor, inverter, battery and
 * vehicle before applying high voltage.
 */
#ifndef EV_CONFIG_H
#define EV_CONFIG_H

/* ------------------------------------------------------------------ */
/* Task timing                                                         */
/* ------------------------------------------------------------------ */
#define PWM_FREQ_HZ                 10000.0f            /* switching / current loop rate */
#define TS_FAST                     (1.0f / PWM_FREQ_HZ)
#define TS_SLOW                     0.001f              /* 1 ms task */
#define VSM_PERIOD_MS               10u                 /* vehicle state machine period */

/* ------------------------------------------------------------------ */
/* Motor: interior permanent magnet synchronous machine (IPMSM)        */
/* ------------------------------------------------------------------ */
#define MOTOR_POLE_PAIRS            4.0f
#define MOTOR_RS_OHM                0.010f              /* stator resistance per phase */
#define MOTOR_LD_H                  0.00012f            /* d-axis inductance */
#define MOTOR_LQ_H                  0.00030f            /* q-axis inductance */
#define MOTOR_PSI_WB                0.060f              /* PM flux linkage (peak, per phase) */
#define MOTOR_MAX_RPM               12000.0f
#define MOTOR_MAX_TORQUE_NM         250.0f
#define MOTOR_MAX_REGEN_NM          150.0f
#define MOTOR_MAX_POWER_W           100000.0f
#define MOTOR_MAX_REGEN_POWER_W     60000.0f
#define MOTOR_OVERSPEED_RPM         13000.0f

/* Resolver electrical offset found at end-of-line calibration [rad] */
#define RESOLVER_OFFSET_RAD         0.0f
/* Resolver pole pairs (1 = one electrical cycle per mechanical rev) */
#define RESOLVER_POLE_PAIRS         1.0f

/* ------------------------------------------------------------------ */
/* Inverter                                                            */
/* ------------------------------------------------------------------ */
#define INV_I_MAX_A                 450.0f              /* current vector magnitude limit (peak) */
#define INV_I_TRIP_A                550.0f              /* software instantaneous phase trip */
#define INV_I_SUM_TRIP_A            40.0f               /* |ia+ib+ic| plausibility limit */
#define INV_I_SUM_TRIP_SAMPLES      5u
#define INV_I_OFFSET_MAX_A          15.0f               /* max acceptable sensor offset at standstill */
#define INV_DEADTIME_NS             1500u
#define INV_DUTY_MIN                0.02f
#define INV_DUTY_MAX                0.98f
#define INV_VOLTAGE_MARGIN          0.96f               /* fraction of linear SVM limit Vdc/sqrt(3) */
#define INV_FW_VOLTAGE_RATIO        0.92f               /* field weakening starts above this */
#define INV_I_FW_MAX_A              300.0f              /* max negative d-axis current for FW */
#define POS_SENSOR_FAULT_SAMPLES    20u

/* Current loop tuning (bandwidth based: Kp = L*wc, Ki = R*wc).
 * ~fs/20: at 12000 rpm the electrical frequency is 800 Hz and the angle
 * advances 0.5 rad per sample; higher bandwidth showed limit cycles in
 * field weakening with the 1.5-sample computation + PWM delay. */
#ifndef CURRENT_LOOP_BW_HZ
#define CURRENT_LOOP_BW_HZ          500.0f
#endif
/* Field weakening voltage loop (output: A, input: V) */
#define FW_KP                       0.5f
#define FW_KI                       400.0f
/* Angle tracking observer (PLL) bandwidth */
#define PLL_BW_HZ                   150.0f
/* Above this speed a disabled inverter uses active short circuit (ASC)
 * instead of freewheel, because back-EMF could exceed the DC link. */
#define ASC_ENTER_RPM               5500.0f
#define ASC_EXIT_RPM                4500.0f

/* ------------------------------------------------------------------ */
/* High-voltage system                                                 */
/* ------------------------------------------------------------------ */
#define HV_OV_TRIP_V                430.0f
#define HV_UV_TRIP_V                220.0f
#define HV_UV_DERATE_V              260.0f
#define HV_SAFE_V                   60.0f               /* touch-safe DC voltage */
#define PRECHARGE_RATIO             0.95f
#define PRECHARGE_MIN_MS            100u
#define PRECHARGE_TIMEOUT_MS        3000u
#define CONTACTOR_DELAY_MS          50u
#define DISCHARGE_TIMEOUT_MS        5000u

/* ------------------------------------------------------------------ */
/* Temperatures                                                        */
/* ------------------------------------------------------------------ */
#define TEMP_INV_DERATE_C           100.0f
#define TEMP_INV_TRIP_C             125.0f
#define TEMP_MOT_DERATE_C           140.0f
#define TEMP_MOT_TRIP_C             165.0f
#define TEMP_TRIP_DEBOUNCE_MS       200u

/* ------------------------------------------------------------------ */
/* Accelerator pedal (two redundant tracks, track 2 = half of track 1) */
/* ------------------------------------------------------------------ */
#define APPS1_MIN_V                 0.80f
#define APPS1_MAX_V                 4.20f
#define APPS2_MIN_V                 0.40f
#define APPS2_MAX_V                 2.10f
#define APPS_OOR_LOW_V              0.25f
#define APPS_OOR_HIGH_V             4.75f
#define APPS_RANGE_DEBOUNCE_MS      20u
#define APPS_PLAUS_TOL              0.10f               /* max track mismatch (normalized) */
#define APPS_PLAUS_DEBOUNCE_MS      100u
#define APPS_DEADBAND               0.03f
#define BTO_PEDAL_ON                0.25f               /* brake-throttle override */
#define BTO_PEDAL_OFF               0.05f

/* ------------------------------------------------------------------ */
/* Drivability                                                         */
/* ------------------------------------------------------------------ */
#define TORQUE_RATE_NMPS            1500.0f             /* torque slew rate */
#define TORQUE_RATE_ZERO_NMPS       400.0f              /* slower near zero (driveline lash) */
#define TORQUE_ZERO_BAND_NM         20.0f
#define REGEN_COAST_NM              30.0f               /* pedal released */
#define REGEN_BRAKE_NM              100.0f              /* brake pedal pressed */
#define REGEN_FADE_LO_RPM           200.0f
#define REGEN_FADE_HI_RPM           800.0f
#define REVERSE_TORQUE_SCALE        0.4f
#define REVERSE_MAX_RPM             2500.0f
#define SPEED_LIMIT_BAND_RPM        300.0f
#define LIMP_TORQUE_SCALE           0.3f
#define GEAR_SHIFT_MAX_RPM          150.0f
#define KEYOFF_MAX_RPM              500.0f              /* key-off above this is ignored until slow */
#define DRIVETRAIN_EFF              0.90f

/* ------------------------------------------------------------------ */
/* Communication                                                       */
/* ------------------------------------------------------------------ */
#define BMS_TIMEOUT_MS              300u

#endif /* EV_CONFIG_H */
