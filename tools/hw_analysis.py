#!/usr/bin/env python3
"""
Hardware design analysis (HVC-AN-xxx) and hardware verification results report.

  python3 tools/hw_analysis.py [--report docs/08_hardware_verification_results.md] [--rev REV]

Reads the hardware design values (tools/hw_design.py) and the firmware's
effective constants (compiled from config/*.h with the host C compiler, plus
the FlexCAN bit timing from src/bsp/mpc5744p/bsp_can.c), runs the design
checks, and reports. Cases that need hardware, labs or manual review are
listed as NOT RUN.

Exit status: 0 if no executed analysis failed.
"""
import argparse
import datetime
import math
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(__file__))
import hw_design as D  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# --------------------------------------------------------------------------
# Firmware constants
# --------------------------------------------------------------------------
FW_NAMES = [
    "ADC_VREF_V", "ADC_FULL_SCALE", "ADC_V_PER_COUNT",
    "CUR_A_PER_COUNT", "CUR_ZERO_COUNTS", "CUR_SENSOR_ZERO_V", "CUR_SENSOR_V_PER_A",
    "CUR_DIV_R_TOP_OHM", "CUR_DIV_R_BOT_OHM",
    "VDC_V_PER_COUNT", "VDC_DIV_R_TOP_OHM", "VDC_DIV_R_BOT_OHM", "VDC_ISO_GAIN", "VDC_OPAMP_GAIN",
    "APPS_V_PER_COUNT", "APPS_DIV_R_TOP_OHM", "APPS_DIV_R_BOT_OHM",
    "NTC_PULLUP_OHM", "NTC_R25_OHM", "NTC_BETA",
    "MOTC_CLK_HZ", "XOSC_HZ", "CAN_CLK_HZ",
    "PWM_FREQ_HZ", "INV_DEADTIME_NS", "INV_DUTY_MAX", "CTU_SAMPLE_DELAY_NS",
    "INV_I_TRIP_A", "INV_I_MAX_A", "INV_I_OFFSET_MAX_A", "CURRENT_LOOP_BW_HZ",
    "HV_OV_TRIP_V", "HV_SAFE_V", "PRECHARGE_TIMEOUT_MS", "PRECHARGE_MIN_MS", "PRECHARGE_RATIO",
    "DISCHARGE_TIMEOUT_MS",
    "APPS1_MIN_V", "APPS1_MAX_V", "APPS2_MIN_V", "APPS2_MAX_V", "APPS_OOR_LOW_V", "APPS_OOR_HIGH_V",
    "TEMP_INV_TRIP_C", "TEMP_MOT_TRIP_C",
]


def firmware_constants():
    src = ['#include <stdio.h>', '#include "board_cfg.h"', '#include "ev_config.h"', "int main(void){"]
    for n in FW_NAMES:
        src.append(f'printf("{n} %.9g\\n", (double)({n}));')
    src.append("return 0;}")
    cc = os.environ.get("CC", "gcc")
    with tempfile.TemporaryDirectory() as tmp:
        c = os.path.join(tmp, "fw.c")
        exe = os.path.join(tmp, "fw")
        with open(c, "w") as f:
            f.write("\n".join(src))
        subprocess.run([cc, "-DHOST_BUILD", "-I", os.path.join(ROOT, "config"), c, "-o", exe],
                       check=True)
        out = subprocess.run([exe], check=True, capture_output=True, text=True).stdout
    fw = {}
    for line in out.splitlines():
        k, v = line.split()
        fw[k] = float(v)

    # FlexCAN CTRL1 fields from the BSP source
    with open(os.path.join(ROOT, "src/bsp/mpc5744p/bsp_can.c")) as f:
        txt = f.read()
    for name in ("PRESDIV", "RJW", "PSEG1", "PSEG2", "PROPSEG"):
        m = re.search(r"\((\d+)uL << \d+\);?\s*/\*\s*" + name + r"\b", txt)
        if not m:
            raise RuntimeError(f"cannot find {name} in bsp_can.c")
        fw["CAN_" + name] = float(m.group(1))
    return fw


# --------------------------------------------------------------------------
# Check bookkeeping
# --------------------------------------------------------------------------
class Case:
    def __init__(self, cid, title, verifies, method="Analysis", fn=None):
        self.id, self.title, self.verifies, self.method, self.fn = cid, title, verifies, method, fn
        self.rows = []          # (criterion, value, limit, ok)
        self.assumed = set()    # names of assumed inputs used

    def check(self, criterion, value, limit, ok):
        self.rows.append((criterion, value, limit, bool(ok)))

    def uses(self, *params):
        """Record inputs; assumed ones make the verdict conditional."""
        for group, key in params:
            p = getattr(D, group)[key]
            if p.source == "assumed":
                self.assumed.add(f"{group}.{key} = {fmt_param(p)} ({p.note})" if p.note else
                                 f"{group}.{key} = {fmt_param(p)}")

    @property
    def verdict(self):
        if self.fn is None:
            return "NOT RUN"
        if any(not r[3] for r in self.rows):
            return "FAIL"
        return "PASS*" if self.assumed else "PASS"


def fmt_param(p):
    return f"{eng(p.value)}{p.unit}"


def eng(x):
    if x == 0:
        return "0 "
    exp = int(math.floor(math.log10(abs(x)) / 3) * 3)
    exp = max(-12, min(9, exp))
    pre = {-12: "p", -9: "n", -6: "µ", -3: "m", 0: "", 3: "k", 6: "M", 9: "G"}[exp]
    return f"{x / 10 ** exp:.4g} {pre}"


def v(group, key):
    return getattr(D, group)[key].value


def par(a, b):
    return a * b / (a + b)


# --------------------------------------------------------------------------
# Analyses
# --------------------------------------------------------------------------
def an_current_scaling(c, fw):
    c.uses(("CURRENT", "i_range"))
    ratio = fw["CUR_DIV_R_BOT_OHM"] / (fw["CUR_DIV_R_TOP_OHM"] + fw["CUR_DIV_R_BOT_OHM"])
    sens_adc = fw["CUR_SENSOR_V_PER_A"] * ratio
    v0 = fw["CUR_SENSOR_ZERO_V"] * ratio
    vref = fw["ADC_VREF_V"]
    i_pos = (vref - v0) / sens_adc
    i_neg = v0 / sens_adc
    a_per_count = (vref / fw["ADC_FULL_SCALE"]) / sens_adc
    need = 1.2 * fw["INV_I_TRIP_A"]
    c.check("Measurable range at ADC (positive)", f"+{i_pos:.0f} A", f"≥ +{need:.0f} A (1.2 × trip)", i_pos >= need)
    c.check("Measurable range at ADC (negative)", f"−{i_neg:.0f} A", f"≥ −{need:.0f} A", i_neg >= need)
    c.check("Sensor range covers trip level", f"±{v('CURRENT', 'i_range'):.0f} A",
            f"≥ ±{need:.0f} A", v("CURRENT", "i_range") >= need)
    c.check("Resolution", f"{a_per_count:.3f} A/count", "≤ 1 A/count", a_per_count <= 1.0)
    c.check("Firmware CUR_A_PER_COUNT matches hardware", f"{fw['CUR_A_PER_COUNT']:.4f}",
            f"{a_per_count:.4f} ± 0.5 %", abs(fw["CUR_A_PER_COUNT"] / a_per_count - 1) < 0.005)
    c.check("Firmware CUR_ZERO_COUNTS matches hardware", f"{fw['CUR_ZERO_COUNTS']:.1f}",
            f"{v0 / (vref / fw['ADC_FULL_SCALE']):.1f} ± 1", abs(fw["CUR_ZERO_COUNTS"] - v0 / (vref / fw["ADC_FULL_SCALE"])) <= 1)


def an_current_filter(c, fw):
    c.uses(("MCU", "adc_cs"))
    r_th = par(fw["CUR_DIV_R_TOP_OHM"], fw["CUR_DIV_R_BOT_OHM"])
    cf = v("CURRENT", "c_filt")
    fc = 1 / (2 * math.pi * r_th * cf)
    lag = math.degrees(math.atan(fw["CURRENT_LOOP_BW_HZ"] / fc))
    ratio = cf / v("MCU", "adc_cs")
    c.check("Filter corner frequency", f"{fc / 1e3:.1f} kHz", "≥ 5 kHz", fc >= 5e3)
    c.check("Phase lag at current-loop bandwidth", f"{lag:.2f}° at {fw['CURRENT_LOOP_BW_HZ']:.0f} Hz", "≤ 5°", lag <= 5)
    c.check("Filter C / ADC sampling C (charge sharing)", f"{ratio:.0f}", "≥ 200", ratio >= 200)


def an_current_offset(c, fw):
    c.uses(("CURRENT", "offset_v"))
    sens = fw["CUR_SENSOR_V_PER_A"]
    limit = 0.6 * fw["INV_I_OFFSET_MAX_A"]
    for key, label in (("supply_tol", "precision 5 V supply (design)"),
                       ("supply_tol_std", "standard ±2 % regulator (rejected alternative)")):
        tol = v("CURRENT", key)
        # ratiometric sensor: zero point = Vsupply / 2
        err_v = fw["CUR_SENSOR_ZERO_V"] * tol + v("CURRENT", "offset_v")
        err_a = err_v / sens
        is_design = key == "supply_tol"
        c.check(f"Worst-case zero offset, {label}", f"{err_a:.1f} A",
                f"≤ {limit:.1f} A (60 % of calibration limit)",
                (err_a <= limit) if is_design else True)
        if not is_design:
            c.rows[-1] = (c.rows[-1][0], c.rows[-1][1],
                          f"info: would {'pass' if err_a <= limit else 'false-trip the offset check'}", True)
    c.uses(("CURRENT", "gain_err"))
    g = v("CURRENT", "gain_err") + v("CURRENT", "supply_tol") + v("ADC_REF", "tol")
    c.check("Worst-case gain error (sensor + supply + reference)", f"{100 * g:.1f} %", "≤ 3 %", g <= 0.03)


def an_vdc(c, fw):
    c.uses(("VDC", "iso_in_fs"), ("VDC", "r_top_vrating"), ("VDC", "r_top_prating"))
    n, r1 = v("VDC", "n_top"), v("VDC", "r_top_each")
    r_top, r_bot = n * r1, fw["VDC_DIV_R_BOT_OHM"]
    g_op = v("VDC", "opamp_rf") / v("VDC", "opamp_rg")       # difference amplifier gain
    k = r_bot / (r_top + r_bot) * fw["VDC_ISO_GAIN"] * g_op       # V at ADC per V DC
    v_fs = fw["ADC_VREF_V"] / k
    v_iso_fs = v("VDC", "iso_in_fs") * (r_top + r_bot) / r_bot
    v_per_count = fw["ADC_VREF_V"] / fw["ADC_FULL_SCALE"] / k
    vmax = 1.15 * fw["HV_OV_TRIP_V"]
    c.check("Firmware VDC_DIV_R_TOP_OHM = string resistance", f"{fw['VDC_DIV_R_TOP_OHM']:.0f} Ω",
            f"{r_top:.0f} Ω", abs(fw["VDC_DIV_R_TOP_OHM"] - r_top) < 1)
    c.check("Firmware VDC_OPAMP_GAIN = Rf/Rg", f"{fw['VDC_OPAMP_GAIN']:.3f}", f"{g_op:.3f}",
            abs(fw["VDC_OPAMP_GAIN"] - g_op) < 1e-3)
    c.check("ADC full scale", f"{v_fs:.0f} V", f"≥ {vmax:.0f} V (1.15 × OV trip)", v_fs >= vmax)
    c.check("Isolated amplifier linear range", f"{v_iso_fs:.0f} V", f"≥ {vmax:.0f} V", v_iso_fs >= vmax)
    c.check("Resolution", f"{v_per_count:.3f} V/count", "≤ 0.5 V/count", v_per_count <= 0.5)
    c.check("Firmware VDC_V_PER_COUNT matches", f"{fw['VDC_V_PER_COUNT']:.4f}", f"{v_per_count:.4f} ± 0.5 %",
            abs(fw["VDC_V_PER_COUNT"] / v_per_count - 1) < 0.005)
    v_each = vmax * r1 / (r_top + r_bot)
    p_each = (vmax / (r_top + r_bot)) ** 2 * r1
    c.check("Voltage per string resistor at 1.15 × OV", f"{v_each:.0f} V",
            f"≤ {0.5 * v('VDC', 'r_top_vrating'):.0f} V (50 % rating)", v_each <= 0.5 * v("VDC", "r_top_vrating"))
    c.check("Power per string resistor", f"{p_each * 1e3:.1f} mW",
            f"≤ {0.5 * v('VDC', 'r_top_prating') * 1e3:.0f} mW", p_each <= 0.5 * v("VDC", "r_top_prating"))


def ntc_v(fw, t):
    r = fw["NTC_R25_OHM"] * math.exp(fw["NTC_BETA"] * (1 / (t + 273.15) - 1 / 298.15))
    return fw["ADC_VREF_V"] * r / (r + fw["NTC_PULLUP_OHM"])


def an_ntc(c, fw):
    lsb = fw["ADC_VREF_V"] / fw["ADC_FULL_SCALE"]
    for name, t_trip in (("inverter", fw["TEMP_INV_TRIP_C"]), ("motor", fw["TEMP_MOT_TRIP_C"])):
        dv = abs(ntc_v(fw, t_trip + 0.5) - ntc_v(fw, t_trip - 0.5))
        res = lsb / dv
        c.check(f"Resolution at {name} trip ({t_trip:.0f} °C)", f"{res:.3f} °C/count", "≤ 0.5 °C/count", res <= 0.5)
    v_lo = ntc_v(fw, v("NTC", "t_max"))
    v_hi = ntc_v(fw, v("NTC", "t_min"))
    c.check(f"Valid range stays above short-circuit threshold (at {v('NTC', 't_max'):.0f} °C)",
            f"{v_lo * 1e3:.0f} mV", "> 50 mV", v_lo > 0.05)
    c.check(f"Valid range stays below open-circuit threshold (at {v('NTC', 't_min'):.0f} °C)",
            f"{v_hi:.3f} V", f"< {fw['ADC_VREF_V'] - 0.05:.2f} V", v_hi < fw["ADC_VREF_V"] - 0.05)
    # tolerance: R25 and pull-up 1 % each, worst direction at the cold end
    r = fw["NTC_R25_OHM"] * 1.01 * math.exp(fw["NTC_BETA"] * (1 / (v("NTC", "t_min") + 273.15) - 1 / 298.15))
    v_hi_tol = fw["ADC_VREF_V"] * r / (r + fw["NTC_PULLUP_OHM"] * 0.99)
    c.check("Same with 1 % tolerances (R25 high, pull-up low)", f"{v_hi_tol:.3f} V",
            f"< {fw['ADC_VREF_V'] - 0.05:.2f} V", v_hi_tol < fw["ADC_VREF_V"] - 0.05)


def an_pedal(c, fw):
    c.uses(("MCU", "inj_max"))
    ratio = fw["APPS_DIV_R_BOT_OHM"] / (fw["APPS_DIV_R_TOP_OHM"] + fw["APPS_DIV_R_BOT_OHM"])
    vref = fw["ADC_VREF_V"]
    lo, hi = fw["APPS_OOR_LOW_V"] * ratio, fw["APPS_OOR_HIGH_V"] * ratio
    c.check("Low OOR threshold at ADC", f"{lo:.3f} V", f"≥ {0.03 * vref:.3f} V (3 % FS)", lo >= 0.03 * vref)
    c.check("High OOR threshold at ADC", f"{hi:.3f} V", f"≤ {0.97 * vref:.3f} V (97 % FS)", hi <= 0.97 * vref)
    vmax_sens = v("PEDAL", "supply") * (1 + v("PEDAL", "supply_tol"))
    c.check("Sensor at supply rail (short to own supply) detected as OOR",
            f"{vmax_sens:.2f} V", f"> {fw['APPS_OOR_HIGH_V']:.2f} V", vmax_sens > fw["APPS_OOR_HIGH_V"])
    vmax_valid = max(fw["APPS1_MAX_V"], fw["APPS2_MAX_V"]) * ratio
    c.check("Highest valid signal inside ADC range", f"{vmax_valid:.3f} V", f"< {vref:.2f} V", vmax_valid < vref)
    i_inj = (v("PEDAL", "short_v") - vref - v("DIN", "v_clamp_diode")) / fw["APPS_DIV_R_TOP_OHM"]
    c.check("Clamp injection with signal shorted to 16 V", f"{i_inj * 1e3:.2f} mA",
            f"≤ {v('MCU', 'inj_max') * 1e3 / 2:.1f} mA (50 % of pin limit)", i_inj <= v("MCU", "inj_max") / 2)
    exp = fw["ADC_V_PER_COUNT"] / ratio
    c.check("Firmware APPS_V_PER_COUNT matches", f"{fw['APPS_V_PER_COUNT'] * 1e3:.4f} mV",
            f"{exp * 1e3:.4f} mV ± 0.5 %", abs(fw["APPS_V_PER_COUNT"] / exp - 1) < 0.005)
    fc = 1 / (2 * math.pi * par(fw["APPS_DIV_R_TOP_OHM"], fw["APPS_DIV_R_BOT_OHM"]) * v("PEDAL", "c_filt"))
    c.check("Input filter corner", f"{fc:.0f} Hz", "50 Hz … 1 kHz (1 ms sampling)", 50 <= fc <= 1000)


def an_din(c, fw):
    c.uses(("MCU", "vih_ratio"), ("MCU", "vil_ratio"), ("MCU", "inj_max"), ("DIN", "r_wet_prating"),
           ("DIN", "v_clamp_diode"))
    vdd = v("MCU", "vdd_io")
    rs, rsh = v("DIN", "r_ser"), v("DIN", "r_sh")
    k = rsh / (rs + rsh)
    vih, vil = v("MCU", "vih_ratio") * vdd, v("MCU", "vil_ratio") * vdd
    vin_hi = v("DIN", "v_high_min") * k
    vin_lo = v("DIN", "v_low_max") * k
    c.check(f"Pin voltage at {v('DIN', 'v_high_min'):.0f} V input", f"{vin_hi:.2f} V", f"≥ VIH {vih:.2f} V", vin_hi >= vih)
    c.check(f"Pin voltage at {v('DIN', 'v_low_max'):.0f} V input", f"{vin_lo:.2f} V", f"≤ VIL {vil:.2f} V", vin_lo <= vil)
    i_wet = v("SUPPLY", "vbat_min") / par(v("DIN", "r_wet"), rs + rsh)
    c.check("Contact wetting current at 9 V", f"{i_wet * 1e3:.1f} mA",
            f"≥ {v('DIN', 'i_wet_min') * 1e3:.0f} mA", i_wet >= v("DIN", "i_wet_min"))
    p_wet = v("SUPPLY", "vbat_jump") ** 2 / v("DIN", "r_wet")
    c.check("Wetting resistor power at 24 V jump start", f"{p_wet:.3f} W",
            f"≤ {v('DIN', 'r_wet_prating') * 1.0:.2f} W (rating; 60 s event)", p_wet <= v("DIN", "r_wet_prating"))
    p_wet16 = v("SUPPLY", "vbat_max") ** 2 / v("DIN", "r_wet")
    c.check("Wetting resistor power at 16 V continuous", f"{p_wet16:.3f} W",
            f"≤ {0.5 * v('DIN', 'r_wet_prating'):.3f} W (50 %)", p_wet16 <= 0.5 * v("DIN", "r_wet_prating"))
    i_inj = (v("SUPPLY", "vbat_jump") - vdd - v("DIN", "v_clamp_diode")) / rs
    c.check("Clamp current at 24 V", f"{i_inj * 1e3:.2f} mA",
            "into Schottky clamp to 3.3 V; MCU pin current ≈ 0", True)
    i_rev = abs(v("SUPPLY", "vbat_rev")) / rs
    c.check("Reverse polarity current into clamp", f"{i_rev * 1e3:.2f} mA", "≤ 1 mA", i_rev <= 1e-3)
    tau = par(rs, rsh) * v("DIN", "c")
    c.check("Input filter time constant", f"{tau * 1e6:.0f} µs", "20 µs … 1 ms", 20e-6 <= tau <= 1e-3)


def an_precharge(c, fw):
    c.uses(("HV", "v_batt_max"), ("HV", "r_pre_energy"), ("HV", "r_pre_burst_energy"))
    cmax = v("HV", "c_dc") * (1 + v("HV", "c_dc_tol"))
    rmax = v("HV", "r_pre") * (1 + v("HV", "r_pre_tol"))
    rmin = v("HV", "r_pre") * (1 - v("HV", "r_pre_tol"))
    t95 = -rmax * cmax * math.log(1 - fw["PRECHARGE_RATIO"])
    c.check("Time to 95 % (max R, max C)", f"{t95 * 1e3:.0f} ms",
            f"≤ {fw['PRECHARGE_TIMEOUT_MS'] / 3:.0f} ms (timeout / 3)", t95 <= fw["PRECHARGE_TIMEOUT_MS"] / 3e3)
    i_pk = v("HV", "v_batt_max") / rmin
    c.check("Peak precharge current", f"{i_pk:.2f} A", f"≤ {v('HV', 'i_pre_max'):.0f} A", i_pk <= v("HV", "i_pre_max"))
    e = 0.5 * cmax * v("HV", "v_batt_max") ** 2
    c.check("Energy in resistor per precharge", f"{e:.1f} J", f"≤ {0.5 * v('HV', 'r_pre_energy'):.0f} J (50 %)",
            e <= 0.5 * v("HV", "r_pre_energy"))
    n = v("HV", "n_precharge_burst")
    c.check(f"Energy for {n:.0f} consecutive precharges", f"{n * e:.0f} J",
            f"≤ {v('HV', 'r_pre_burst_energy'):.0f} J", n * e <= v("HV", "r_pre_burst_energy"))


def an_active_dis(c, fw):
    c.uses(("HV", "r_dis_energy"))
    cmax = v("HV", "c_dc") * (1 + v("HV", "c_dc_tol"))
    v0 = fw["HV_OV_TRIP_V"]
    t = v("HV", "r_dis") * cmax * math.log(v0 / fw["HV_SAFE_V"])
    c.check(f"Time {v0:.0f} V → {fw['HV_SAFE_V']:.0f} V", f"{t * 1e3:.0f} ms",
            f"≤ {v('HV', 't_dis_active_max') * 1e3:.0f} ms", t <= v("HV", "t_dis_active_max"))
    c.check("Firmware DISCHARGE_TIMEOUT_MS ≥ 3 × discharge time", f"{fw['DISCHARGE_TIMEOUT_MS']:.0f} ms",
            f"≥ {3 * t * 1e3:.0f} ms", fw["DISCHARGE_TIMEOUT_MS"] >= 3 * t * 1e3)
    e = 0.5 * cmax * v0 ** 2
    c.check("Energy per discharge", f"{e:.1f} J", f"≤ {0.5 * v('HV', 'r_dis_energy'):.0f} J (50 %)",
            e <= 0.5 * v("HV", "r_dis_energy"))
    c.check("Peak discharge current", f"{v0 / v('HV', 'r_dis'):.1f} A", "switch rating ≥ 2 ×, see review", True)


def an_passive_dis(c, fw):
    c.uses(("HV", "r_bleed_prating"), ("HV", "r_bleed_vrating"))
    n, r1 = v("HV", "n_bleed"), v("HV", "r_bleed_each")
    r = n * r1
    cmax = v("HV", "c_dc") * (1 + v("HV", "c_dc_tol"))
    v0 = fw["HV_OV_TRIP_V"]
    t = r * cmax * math.log(v0 / fw["HV_SAFE_V"])
    c.check(f"Time {v0:.0f} V → {fw['HV_SAFE_V']:.0f} V without supply", f"{t:.0f} s",
            f"≤ {v('HV', 't_dis_passive_max'):.0f} s", t <= v("HV", "t_dis_passive_max"))
    p_each = v0 ** 2 / r / n
    c.check("Continuous power per resistor at OV trip", f"{p_each:.3f} W",
            f"≤ {0.5 * v('HV', 'r_bleed_prating'):.2f} W (50 %)", p_each <= 0.5 * v("HV", "r_bleed_prating"))
    v_each = v0 / n
    c.check("Voltage per resistor", f"{v_each:.0f} V", f"≤ {0.75 * v('HV', 'r_bleed_vrating'):.0f} V (75 %)",
            v_each <= 0.75 * v("HV", "r_bleed_vrating"))
    c.check("Continuous loss", f"{v0 ** 2 / r:.2f} W", "info", True)


def an_dclink(c, fw):
    vr = v("HV", "c_dc_vrating")
    need = 1.05 * fw["HV_OV_TRIP_V"]
    c.check("DC link capacitor voltage rating", f"{vr:.0f} V", f"≥ {need:.0f} V (1.05 × OV trip)", vr >= need)


def an_module(c, fw):
    c.uses(("MODULE", "v_ces"), ("MODULE", "i_c"))
    c.check("Blocking voltage vs OV trip", f"{v('MODULE', 'v_ces'):.0f} V",
            f"≥ {1.6 * fw['HV_OV_TRIP_V']:.0f} V (1.6 × OV trip, overshoot margin)",
            v("MODULE", "v_ces") >= 1.6 * fw["HV_OV_TRIP_V"])
    c.check("Current rating vs software trip", f"{v('MODULE', 'i_c'):.0f} A",
            f"≥ {1.4 * fw['INV_I_TRIP_A']:.0f} A (1.4 × trip)", v("MODULE", "i_c") >= 1.4 * fw["INV_I_TRIP_A"])


def an_pwm_timing(c, fw):
    c.uses(("MODULE", "td_off_tf_max"), ("GATE", "tpd_skew"))
    mod = fw["MOTC_CLK_HZ"] / fw["PWM_FREQ_HZ"]
    c.check("PWM period in counts is an integer", f"{mod:.3f}", "integer", abs(mod - round(mod)) < 1e-6)
    c.check("Half period fits signed 16-bit", f"{mod / 2:.0f}", "≤ 32767", mod / 2 <= 32767)
    dt_cnt = fw["INV_DEADTIME_NS"] * 1e-9 * fw["MOTC_CLK_HZ"]
    c.check("Dead time in counts", f"{dt_cnt:.2f}", "integer, ≤ 2047", abs(dt_cnt - round(dt_cnt)) < 1e-6 and dt_cnt <= 2047)
    need = v("MODULE", "td_off_tf_max") + v("GATE", "tpd_skew")
    c.check("Dead time vs switch turn-off + driver skew", f"{fw['INV_DEADTIME_NS'] / 1e3:.2f} µs",
            f"≥ {need * 1e6:.2f} µs", fw["INV_DEADTIME_NS"] * 1e-9 >= need)
    t_edge = (1 - fw["INV_DUTY_MAX"]) / 2 / fw["PWM_FREQ_HZ"] + fw["CTU_SAMPLE_DELAY_NS"] * 1e-9
    c.check("Last switching edge → current sample (max duty)", f"{t_edge * 1e6:.2f} µs",
            f"≥ {v('GATE', 't_sense_after_edge_min') * 1e6:.1f} µs", t_edge >= v("GATE", "t_sense_after_edge_min"))


def an_can(c, fw):
    pres = fw["CAN_PRESDIV"] + 1
    prop, ps1, ps2, rjw = fw["CAN_PROPSEG"] + 1, fw["CAN_PSEG1"] + 1, fw["CAN_PSEG2"] + 1, fw["CAN_RJW"] + 1
    ntq = 1 + prop + ps1 + ps2
    rate = fw["CAN_CLK_HZ"] / pres / ntq
    sp = (1 + prop + ps1) / ntq
    c.check("Bit rate", f"{rate / 1e3:.1f} kbit/s", "500 kbit/s", abs(rate - 500e3) < 1)
    c.check("Sample point", f"{100 * sp:.1f} %", "85 … 90 % (automotive practice)", 0.85 <= sp <= 0.90)
    c.check("SJW ≤ PSEG2", f"{rjw:.0f} ≤ {ps2:.0f}", "required", rjw <= ps2)
    # ISO 11898-1 oscillator tolerance conditions
    df1 = min(ps1, ps2) / (2 * (13 * ntq - ps2))
    df2 = rjw / (20 * ntq)
    df = min(df1, df2)
    c.check("Allowed oscillator tolerance (ISO 11898-1)", f"±{df * 1e6:.0f} ppm",
            f"≥ ±{v('CAN', 'osc_tol') * 1e6:.0f} ppm (crystal)", df >= v("CAN", "osc_tol"))


def an_lv_budget(c, fw):
    c.uses(("MCU", "i_33"), ("LV", "i_rdc"), ("LV", "i_can"), ("LV", "i_pedal_sensor"), ("LV", "i_misc_33"),
           ("LV", "i_misc_5"), ("CURRENT", "i_supply"), ("LV", "ldo33_theta_ja"), ("LV", "tj_max"),
           ("MODULE", "qg"), ("LV", "eff_iso"))
    i33 = v("MCU", "i_33") + v("LV", "i_misc_33")
    c.check("3.3 V LDO loading", f"{i33:.2f} A", f"≤ {v('LV', 'load_max') * v('LV', 'ldo33_rating'):.2f} A",
            i33 <= v("LV", "load_max") * v("LV", "ldo33_rating"))
    p_ldo = (5.0 - 3.3) * i33
    tj = v("LV", "t_amb_max") + p_ldo * v("LV", "ldo33_theta_ja")
    c.check("3.3 V LDO junction temperature at 105 °C ambient", f"{tj:.0f} °C",
            f"≤ {v('LV', 'tj_max') - 10:.0f} °C", tj <= v("LV", "tj_max") - 10)
    i5 = (i33 + v("LV", "i_rdc") + v("LV", "i_can") + 3 * v("CURRENT", "i_supply")
          + 2 * v("LV", "i_pedal_sensor") + v("LV", "i_misc_5"))
    c.check("5 V main buck loading", f"{i5:.2f} A", f"≤ {v('LV', 'load_max') * v('LV', 'buck5_rating'):.2f} A",
            i5 <= v("LV", "load_max") * v("LV", "buck5_rating"))
    p_gate = v("MODULE", "qg") * (v("GATE", "v_pos") - v("GATE", "v_neg")) * fw["PWM_FREQ_HZ"]
    c.check("Gate drive power per switch", f"{p_gate:.2f} W",
            f"≤ {v('LV', 'load_max') * v('GATE', 'iso_supply_w'):.2f} W (isolated supply)",
            p_gate <= v("LV", "load_max") * v("GATE", "iso_supply_w"))
    i_gd = 6 * (p_gate + 0.15) / v("LV", "eff_iso") / 5.0   # + 0.15 W driver quiescent per channel
    c.check("5 V gate-drive buck loading (6 channels)", f"{i_gd:.2f} A",
            f"≤ {v('LV', 'load_max') * v('LV', 'buck5gd_rating'):.2f} A",
            i_gd <= v("LV", "load_max") * v("LV", "buck5gd_rating"))


# --------------------------------------------------------------------------
# Registry (must match docs/07_hardware_verification_cases.md, see tools/check_trace.sh)
# --------------------------------------------------------------------------
CASES = [
    Case("HVC-AN-001", "Phase current measurement range and scaling", "HWR-SNS-001", fn=an_current_scaling),
    Case("HVC-AN-002", "Phase current filter and ADC loading", "HWR-SNS-001", fn=an_current_filter),
    Case("HVC-AN-003", "Phase current offset and gain error budget", "HWR-SNS-002, HWR-PWR-004", fn=an_current_offset),
    Case("HVC-AN-004", "DC link voltage measurement", "HWR-SNS-003", fn=an_vdc),
    Case("HVC-AN-005", "Temperature measurement", "HWR-SNS-004", fn=an_ntc),
    Case("HVC-AN-006", "Accelerator pedal inputs", "HWR-IF-001", fn=an_pedal),
    Case("HVC-AN-007", "Digital switch inputs", "HWR-IF-002", fn=an_din),
    Case("HVC-AN-008", "Precharge circuit", "HWR-HV-002", fn=an_precharge),
    Case("HVC-AN-009", "Active discharge", "HWR-HV-003", fn=an_active_dis),
    Case("HVC-AN-010", "Passive discharge", "HWR-HV-004", fn=an_passive_dis),
    Case("HVC-AN-011", "DC link capacitor rating", "HWR-HV-005", fn=an_dclink),
    Case("HVC-AN-012", "Power module ratings", "HWR-PS-001", fn=an_module),
    Case("HVC-AN-013", "PWM, dead time and sampling timing", "HWR-PS-002, HWR-SNS-006", fn=an_pwm_timing),
    Case("HVC-AN-014", "CAN bit timing and oscillator tolerance", "HWR-IF-005, HWR-MCU-001", fn=an_can),
    Case("HVC-AN-015", "Low-voltage power budget", "HWR-PWR-005, HWR-GD-001", fn=an_lv_budget),

    Case("HVC-RV-001", "Schematic review against data sheets",
         "HWR-PWR-002, HWR-PWR-003, HWR-MCU-001, HWR-MCU-002, HWR-MCU-003, HWR-GD-001, HWR-GD-002, "
         "HWR-GD-003, HWR-GD-004, HWR-SNS-005, HWR-IF-003, HWR-IF-004", "Review"),
    Case("HVC-RV-002", "Layout review: HV/LV separation, creepage and clearance", "HWR-ISO-001", "Review"),
    Case("HVC-RV-003", "Component qualification review", "HWR-ENV-001", "Review"),

    Case("HVC-BN-001", "Low-voltage power-up, supply rails and supply transients",
         "HWR-PWR-001, HWR-PWR-002, HWR-PWR-003, HWR-PWR-004, HWR-MCU-003", "Bench test"),
    Case("HVC-BN-002", "Clock and debug interface", "HWR-MCU-001, HWR-MCU-002", "Bench test"),
    Case("HVC-BN-003", "Analog front-end accuracy",
         "HWR-SNS-001, HWR-SNS-002, HWR-SNS-003, HWR-SNS-004", "Bench test"),
    Case("HVC-BN-004", "Resolver interface", "HWR-SNS-005", "Bench test"),
    Case("HVC-BN-005", "Pedal and digital input thresholds", "HWR-IF-001, HWR-IF-002", "Bench test"),
    Case("HVC-BN-006", "Low-side outputs with contactor loads", "HWR-IF-003, HWR-HV-001", "Bench test"),
    Case("HVC-BN-007", "CAN physical layer", "HWR-IF-004", "Bench test"),
    Case("HVC-BN-008", "Gate drive waveforms and double-pulse test",
         "HWR-GD-001, HWR-GD-004, HWR-PS-002", "Bench test"),
    Case("HVC-BN-009", "Desaturation short-circuit test", "HWR-GD-002, HWR-GD-003", "Bench test"),
    Case("HVC-BN-010", "High-voltage precharge and discharge",
         "HWR-HV-001, HWR-HV-002, HWR-HV-003, HWR-HV-004", "Bench test"),

    Case("HVC-EN-001", "Operating temperature and thermal cycling", "HWR-ENV-001", "Environmental test"),
    Case("HVC-EN-002", "EMC emissions and immunity", "HWR-ENV-002", "Environmental test"),
    Case("HVC-EN-003", "Electrostatic discharge", "HWR-ENV-003", "Environmental test"),
    Case("HVC-EN-004", "Vibration and mechanical shock", "HWR-ENV-004", "Environmental test"),
    Case("HVC-EN-005", "Dielectric withstand and insulation resistance", "HWR-ISO-001", "Environmental test"),
]


def md(s):
    return str(s).replace("|", "\\|")


def write_report(path, rev, fw):
    now = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    run = [c for c in CASES if c.fn]
    n_pass = sum(c.verdict == "PASS" for c in run)
    n_passc = sum(c.verdict == "PASS*" for c in run)
    n_fail = sum(c.verdict == "FAIL" for c in run)
    nr = [c for c in CASES if not c.fn]
    L = []
    L.append("# Hardware Verification Results\n")
    L.append("> Generated by `make hw-report` (`tools/hw_analysis.py`). Do not edit by hand.\n")
    L.append("| Item | Value |\n|---|---|")
    L.append("| Document | HVR, results for the cases in [07_hardware_verification_cases.md](07_hardware_verification_cases.md) |")
    L.append(f"| Design revision | `{rev}` |")
    L.append(f"| Date | {now} |")
    L.append("| Inputs | `tools/hw_design.py` (component values), `config/*.h` and `bsp_can.c` (firmware constants, compiled) |")
    L.append("| Hardware | None built. Only design analyses were executed. |\n")
    L.append("## Summary\n")
    L.append("| | Cases |\n|---|---|")
    L.append(f"| Analysis executed | {len(run)} |")
    L.append(f"| PASS | {n_pass} |")
    L.append(f"| PASS* (depends on data sheet values not yet confirmed) | {n_passc} |")
    L.append(f"| FAIL | {n_fail} |")
    for m in ("Review", "Bench test", "Environmental test"):
        L.append(f"| Not run: {m.lower()} | {sum(c.method == m for c in nr)} |")
    L.append(f"| Total | {len(CASES)} |\n")
    if n_fail:
        L.append("**Verdict: FAILED.** See failing checks below.\n")
    else:
        L.append("**Verdict:** the design passes all executed analyses, partly on assumed data sheet values "
                 "(PASS*). **No hardware has been built or tested**: every requirement that needs a bench, "
                 "environmental or review case remains unverified. These results show the design calculations "
                 "are consistent, not that the hardware works.\n")
    L.append("## Analysis results\n")
    for c in run:
        L.append(f"### {c.id} {c.title}: **{c.verdict}**\n")
        L.append(f"Verifies: {c.verifies}\n")
        L.append("| Criterion | Value | Limit | OK |\n|---|---|---|---|")
        for crit, val, lim, ok in c.rows:
            L.append(f"| {md(crit)} | {md(val)} | {md(lim)} | {'yes' if ok else '**NO**'} |")
        if c.assumed:
            L.append("\nAssumed inputs (confirm against the selected part's data sheet):\n")
            for a in sorted(c.assumed):
                L.append(f"* `{a}`")
        L.append("")
    L.append("## Not executed\n")
    L.append("| Case | Title | Verifies | Method | Reason |\n|---|---|---|---|---|")
    for c in nr:
        reason = {"Review": "Manual review not yet performed",
                  "Bench test": "No hardware built",
                  "Environmental test": "No hardware built; needs accredited lab"}[c.method]
        L.append(f"| {c.id} | {md(c.title)} | {c.verifies} | {c.method} | {reason} |")
    L.append("")
    with open(path, "w") as f:
        f.write("\n".join(L))
    print(f"report written to {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report")
    ap.add_argument("--rev", default="unknown")
    ap.add_argument("--list", action="store_true", help="print the case registry (id|title|verifies)")
    a = ap.parse_args()
    if a.list:
        for c in CASES:
            print(f"{c.id}|{c.title}|{c.verifies}")
        return 0
    fw = firmware_constants()
    for c in CASES:
        if c.fn:
            c.fn(c, fw)
            print(f"[{c.verdict:7}] {c.id} {c.title}")
            for crit, val, lim, ok in c.rows:
                if not ok:
                    print(f"      FAIL: {crit}: {val} (limit {lim})")
        else:
            print(f"[NOT RUN] {c.id} {c.title} ({c.method})")
    if a.report:
        write_report(a.report, a.rev, fw)
    return 1 if any(c.verdict == "FAIL" for c in CASES) else 0


if __name__ == "__main__":
    sys.exit(main())
