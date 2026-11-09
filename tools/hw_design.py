"""
Hardware design values: the single source for the schematics
(tools/draw_schematics.py) and the design analysis (tools/hw_analysis.py).

Each value is a Param with a source:
  "design"     chosen by this design (resistor values, ratings to buy)
  "assumed"    typical datasheet-class value NOT yet confirmed against the
               actual part's datasheet; analysis results that depend on it
               are marked PASS* until the value is confirmed
  "requirement" a limit from the requirements

Firmware constants (config/*.h, bsp_can.c) are NOT duplicated here: the
analysis reads them from the source files and checks consistency.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Param:
    value: float
    unit: str
    source: str
    note: str = ""


P = Param

# ---------------------------------------------------------------- supply
SUPPLY = {
    "vbat_min": P(9.0, "V", "requirement", "lowest functional battery voltage"),
    "vbat_max": P(16.0, "V", "requirement", "highest continuous battery voltage"),
    "vbat_jump": P(24.0, "V", "requirement", "jump start, 60 s"),
    "vbat_rev": P(-14.0, "V", "requirement", "reverse polarity, 60 s"),
}

# ---------------------------------------------------------------- MCU
MCU = {
    "vdd_io": P(3.3, "V", "design", "VDD_HV_IO"),
    "vih_ratio": P(0.65, "", "assumed", "VIH min as fraction of VDD_HV_IO"),
    "vil_ratio": P(0.35, "", "assumed", "VIL max as fraction of VDD_HV_IO"),
    "inj_max": P(2.0e-3, "A", "assumed", "max DC injection current per pin"),
    "adc_cs": P(5e-12, "F", "assumed", "ADC sampling capacitance"),
    "i_33": P(0.40, "A", "assumed", "MPC5744P total from 3.3 V incl. core via ballast"),
}

ADC_REF = {
    "tol": P(0.005, "", "design", "precision 3.3 V analog supply / VREFH tolerance"),
}

# ---------------------------------------------------------------- phase current
CURRENT = {
    "i_range": P(800.0, "A", "assumed", "sensor nominal range (open-loop Hall, 5 V)"),
    "supply_tol": P(0.005, "", "design", "precision 5 V sensor supply tolerance"),
    "supply_tol_std": P(0.02, "", "design", "standard regulator tolerance (comparison)"),
    "offset_v": P(0.008, "V", "assumed", "sensor electrical offset incl. drift"),
    "gain_err": P(0.01, "", "assumed", "sensor sensitivity error"),
    "c_filt": P(2.2e-9, "F", "design", "C at ADC pin"),
    "i_supply": P(0.020, "A", "assumed", "per sensor"),
}

# ---------------------------------------------------------------- DC link voltage
VDC = {
    "n_top": P(5, "", "design", "series resistors in the HV string"),
    "r_top_each": P(1e6, "ohm", "design", "1206, 1 %"),
    "r_top_vrating": P(200.0, "V", "assumed", "1206 working voltage"),
    "r_top_prating": P(0.25, "W", "assumed", "1206 power rating"),
    "iso_in_fs": P(2.0, "V", "assumed", "isolated amplifier linear input range"),
    "opamp_rf": P(15e3, "ohm", "design"),
    "opamp_rg": P(10e3, "ohm", "design"),
}

# ---------------------------------------------------------------- NTC
NTC = {
    "t_min": P(-40.0, "C", "requirement"),
    "t_max": P(175.0, "C", "requirement"),
}

# ---------------------------------------------------------------- pedal
PEDAL = {
    "supply": P(5.0, "V", "design", "separate 5 V sensor supply per track"),
    "supply_tol": P(0.02, "", "design"),
    "c_filt": P(100e-9, "F", "design"),
    "short_v": P(16.0, "V", "requirement", "signal shorted to battery"),
}

# ---------------------------------------------------------------- digital inputs
DIN = {
    "r_wet": P(2200.0, "ohm", "design", "2512 (1206 overloaded at 24 V jump start)"),
    "r_wet_prating": P(1.0, "W", "assumed", "2512 power rating"),
    "r_ser": P(22000.0, "ohm", "design", "0603"),
    "r_sh": P(10000.0, "ohm", "design", "0603"),
    "c": P(10e-9, "F", "design"),
    "v_clamp_diode": P(0.3, "V", "assumed", "Schottky clamp to 3.3 V"),
    "v_high_min": P(8.0, "V", "requirement", "input must be HIGH at/above"),
    "v_low_max": P(2.0, "V", "requirement", "input must be LOW at/below"),
    "i_wet_min": P(2e-3, "A", "requirement", "contact wetting current at 9 V"),
}

# ---------------------------------------------------------------- HV path
HV = {
    "c_dc": P(500e-6, "F", "design", "film capacitor"),
    "c_dc_tol": P(0.10, "", "design"),
    "c_dc_vrating": P(500.0, "V", "design"),
    "v_batt_max": P(403.0, "V", "assumed", "battery max voltage (96s x 4.2 V)"),
    "r_pre": P(50.0, "ohm", "design", "pulse rated"),
    "r_pre_tol": P(0.05, "", "design"),
    "r_pre_energy": P(100.0, "J", "assumed", "single pulse energy rating"),
    "n_precharge_burst": P(5, "", "requirement", "consecutive precharges"),
    "r_pre_burst_energy": P(300.0, "J", "assumed", "rating for a 5-pulse burst"),
    "i_pre_max": P(10.0, "A", "requirement", "peak precharge current"),
    "r_dis": P(50.0, "ohm", "design", "pulse rated"),
    "r_dis_energy": P(200.0, "J", "assumed", "single pulse energy rating"),
    "t_dis_active_max": P(1.0, "s", "requirement"),
    "n_bleed": P(4, "", "design", "series resistors"),
    "r_bleed_each": P(25e3, "ohm", "design", "2512"),
    "r_bleed_prating": P(1.0, "W", "assumed", "2512 power rating"),
    "r_bleed_vrating": P(200.0, "V", "assumed", "2512 working voltage"),
    "t_dis_passive_max": P(120.0, "s", "requirement"),
    "v_safe": P(60.0, "V", "requirement"),
}

# ---------------------------------------------------------------- power stage
MODULE = {
    "v_ces": P(750.0, "V", "assumed", "six-pack IGBT module, e.g. HybridPACK Drive class"),
    "i_c": P(820.0, "A", "assumed", "nominal collector current"),
    "td_off_tf_max": P(1.0e-6, "s", "assumed", "turn-off delay + fall time, max, at 150 C"),
    "qg": P(6.0e-6, "C", "assumed", "total gate charge at +15/-5 V"),
}

GATE = {
    "v_pos": P(15.0, "V", "design"),
    "v_neg": P(-5.0, "V", "design"),
    "tpd_skew": P(30e-9, "s", "assumed", "driver part-to-part propagation skew"),
    "iso_supply_w": P(2.0, "W", "assumed", "per-driver isolated supply rating"),
    "t_sense_after_edge_min": P(1.0e-6, "s", "requirement",
                                "min time from last switching edge to current sample"),
}

# ---------------------------------------------------------------- LV power budget
LV = {
    "buck5_rating": P(3.0, "A", "design", "5 V main buck"),
    "buck5gd_rating": P(3.0, "A", "design", "5 V gate-drive buck"),
    "ldo33_rating": P(1.0, "A", "design", "3.3 V digital LDO"),
    "ldo33_theta_ja": P(40.0, "K/W", "assumed"),
    "t_amb_max": P(105.0, "C", "requirement"),
    "tj_max": P(150.0, "C", "assumed"),
    "i_rdc": P(0.05, "A", "assumed", "AD2S1210 + excitation buffer from 5 V (average)"),
    "i_can": P(0.07, "A", "assumed", "CAN transceiver dominant, from 5 V"),
    "i_pedal_sensor": P(0.015, "A", "assumed", "per track"),
    "i_misc_33": P(0.05, "A", "assumed", "logic, LEDs, pull-ups"),
    "i_misc_5": P(0.05, "A", "assumed"),
    "eff_iso": P(0.80, "", "assumed", "isolated gate supply efficiency"),
    "load_max": P(0.70, "", "requirement", "max regulator loading"),
}

CAN = {
    "osc_tol": P(100e-6, "", "design", "crystal incl. temperature and aging"),
}

# ---------------------------------------------------------------- BOM (concept)
# (reference, function, candidate part, status)
BOM = [
    ("U1", "Microcontroller", "NXP MPC5744P (257 MAPBGA)", "selected"),
    ("Y1", "Crystal 40 MHz, ±50 ppm, AEC-Q200", "to be selected", "open"),
    ("U2", "5 V main buck, VIN ≥ 42 V, 3 A, AEC-Q100", "to be selected", "open"),
    ("U3", "5 V gate-drive buck, VIN ≥ 42 V, 3 A, AEC-Q100", "to be selected", "open"),
    ("U4", "3.3 V digital LDO, 1 A, AEC-Q100", "to be selected", "open"),
    ("U5", "3.3 V analog supply / ADC reference, ±0.5 %", "to be selected", "open"),
    ("U6", "5.000 V precision current-sensor supply, ±0.5 %, ≥ 100 mA", "to be selected", "open"),
    ("U7, U8", "5 V pedal sensor supplies, short-circuit protected (tracking LDO)", "to be selected", "open"),
    ("U9", "Resolver-to-digital converter", "Analog Devices AD2S1210", "verify"),
    ("U10", "Resolver excitation buffer (push-pull op amp)", "per AD2S1210 data sheet application", "open"),
    ("U11", "2-input AND (LOT·DOS → RDC_FAULT)", "74LVC1G08", "verify"),
    ("U12", "CAN FD transceiver with VIO", "TI TCAN1044AV-Q1", "verify"),
    ("U13–U18", "Isolated gate driver with DESAT, Miller clamp", "TI UCC21750-Q1", "verify"),
    ("U19–U24", "Push-pull transformer driver for isolated gate supply", "TI SN6505B-Q1", "verify"),
    ("U25", "Reinforced isolated amplifier, DC link voltage", "TI AMC1311-Q1", "verify"),
    ("U26", "Op amp, differential to single-ended", "rail-to-rail, AEC-Q100, to be selected", "open"),
    ("U27", "Reverse polarity ideal-diode controller", "TI LM74700-Q1 + N-MOSFET", "verify"),
    ("U28–U32", "Protected low-side switch ≥ 3 A (contactors, relay, discharge driver)", "to be selected", "open"),
    ("PM1", "Six-pack IGBT power module 750 V / 820 A", "Infineon HybridPACK Drive class", "verify"),
    ("CS1–CS3", "Open-loop Hall current sensor ±800 A, 5 V", "e.g. LEM HAH1DR series", "verify"),
    ("C_DC", "DC link film capacitor 500 µF / 500 V", "to be selected", "open"),
    ("R_PRE", "Pulse-rated resistor 50 Ω, ≥ 100 J single pulse, ≥ 300 J / 5 pulses", "to be selected", "open"),
    ("R_DIS", "Pulse-rated resistor 50 Ω, ≥ 200 J single pulse", "to be selected", "open"),
    ("Q_DIS", "Discharge switch 1200 V with isolated drive", "to be selected", "open"),
    ("D_ESD", "CAN bus ESD protection", "TI ESD2CAN24-Q1", "verify"),
    ("L_CM", "CAN common-mode choke 51 µH", "TDK ACT45B-510-2P", "verify"),
]
