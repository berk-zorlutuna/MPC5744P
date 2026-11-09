#!/usr/bin/env python3
"""
Draws the concept schematics into docs/hw/*.svg.

  pip install schemdraw==0.19 matplotlib   (matplotlib only needed for --png)
  python3 tools/draw_schematics.py [--png OUTDIR]

Component values are taken from tools/hw_design.py and from the firmware
constants in config/*.h (via tools/hw_analysis.py), so the drawings, the
design analysis and the firmware cannot silently disagree.
"""
import argparse
import os
import sys

import schemdraw
import schemdraw.elements as elm

sys.path.insert(0, os.path.dirname(__file__))
import hw_design as D  # noqa: E402
from hw_analysis import firmware_constants  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUT = os.path.join(ROOT, "docs", "hw")
FW = firmware_constants()


def ohm(r):
    if r >= 1e6:
        return f"{r / 1e6:g} MΩ"
    if r >= 1e3:
        return f"{r / 1e3:g} kΩ"
    return f"{r:g} Ω"


def farad(c):
    if c >= 1e-6:
        return f"{c * 1e6:g} µF"
    if c >= 1e-9:
        return f"{c * 1e9:g} nF"
    return f"{c * 1e12:g} pF"


def val(group, key):
    return getattr(D, group)[key].value


def Rect(corner1, corner2, **kw):
    """Rectangle in absolute coordinates (elm.Rect is relative to the pen)."""
    return elm.Rect(corner1=corner1, corner2=corner2, **kw).at((0, 0)).theta(0)


def Ic(**kw):
    """IC block with fixed orientation (schemdraw otherwise inherits the last direction)."""
    return elm.Ic(**kw).theta(0)


def new_drawing():
    d = schemdraw.Drawing(show=False)
    d.config(fontsize=11, unit=2.5, lw=1.2)
    return d


def title(d, x, y, text, sub=None):
    d.add(elm.Label().at((x, y)).label(text, fontsize=16, halign="left"))
    if sub:
        d.add(elm.Label().at((x, y - 0.8)).label(sub, fontsize=10, halign="left", color="#555555"))


def net(d, xy, name, side="right"):
    """Off-sheet connector: open dot with net name."""
    d.add(elm.Dot(open=True).at(xy).label(name, loc=side, fontsize=10))


def note(d, xy, text, size=9):
    d.add(elm.Label().at(xy).label(text, fontsize=size, halign="left", color="#444444"))


def wire(d, *pts):
    for a, b in zip(pts, pts[1:]):
        d.add(elm.Line().at(a).to(b))


def dot(d, xy):
    d.add(elm.Dot().at(xy))


# ==========================================================================
# Sheet 0: block diagram
# ==========================================================================
def block(d, x, y, w, h, text, fill="#eef4fb"):
    d.add(Rect((x, y), corner2=(x + w, y + h), fill=fill))
    d.add(elm.Label().at((x + w / 2, y + h / 2)).label(text, fontsize=10, halign="center", valign="center"))


def arrow(d, a, b, text=None, loc="top"):
    e = elm.Arrow().at(a).to(b)
    if text:
        e = e.label(text, loc=loc, fontsize=9)
    d.add(e)


def route(d, pts, text=None, loc="right"):
    """Orthogonal arrow along pts; arrowhead on the last segment."""
    for p0, p1 in zip(pts[:-2], pts[1:-1]):
        d.add(elm.Line().at(p0).to(p1))
    e = elm.Arrow().at(pts[-2]).to(pts[-1])
    d.add(e)
    if text:
        mx = (pts[0][0] + pts[1][0]) / 2
        my = (pts[0][1] + pts[1][1]) / 2
        d.add(elm.Label().at((mx + (0.25 if loc == "right" else -0.25), my))
              .label(text, fontsize=9, halign="left" if loc == "right" else "right"))


def sheet0():
    d = new_drawing()
    title(d, 0, 24.5, "Sheet 0  System block diagram",
          "Blocks above the dashed line are at high voltage. Gate drivers, the V_DC amplifier, the current "
          "sensors and the resolver provide the isolation.")
    hv, iso = "#fdeeee", "#fff4e0"
    block(d, 0, 15, 5, 5, "HV battery\n+ BMS\n(external)", hv)
    block(d, 7, 15, 6, 5, "Contactors\nHV+ / HV− /\nprecharge\n(sheet 1)", hv)
    block(d, 15, 15, 5, 5, "DC link\n500 µF\ndischarge\n(sheet 1)", hv)
    block(d, 22, 15, 6, 5, "IGBT six-pack\n750 V / 820 A\n(sheet 1)", hv)
    block(d, 30, 15, 5, 5, "IPMSM\n+ resolver\n+ NTC", hv)
    block(d, 15, 9, 5, 4, "V_DC isolated\namplifier\n(sheet 3)", iso)
    block(d, 22, 9, 6, 4, "Gate drivers ×6\nisolated, DESAT\n(sheet 2)", iso)
    block(d, 30, 9, 3.4, 4, "Hall current\nsensors ×3\n(sheet 3)", iso)
    block(d, 2, 0, 8, 6, "LV power supply\n12 V → 5 V, 3V3,\n3V3_A, 5V_PREC,\n5V_SENS ×2\n(sheet 6)")
    block(d, 12, 0, 15, 6, "MPC5744P\n200 MHz lockstep core\nFlexPWM · CTU · ADC ×4\nDSPI · FlexCAN · SWT\n(sheet 4)", "#e7f5e7")
    block(d, 29, 0, 8, 6, "Vehicle interfaces\nkey, brake, P/R/N/D,\npedal ×2, CAN,\nresolver RDC,\ncontactor drivers\n(sheet 5)")
    # power path
    for x0, x1 in ((5, 7), (13, 15), (20, 22), (28, 30)):
        route(d, [(x0, 17.5), (x1, 17.5)])
    d.add(elm.Label().at((29, 18.0)).label("U V W", fontsize=9))
    # sensing / control
    route(d, [(17.5, 15), (17.5, 13)])
    route(d, [(17.5, 9), (17.5, 6)], "V_DC")
    route(d, [(23.0, 6), (23.0, 9)], "PWM ×6, EN", "left")
    route(d, [(25.0, 9), (25.0, 6)], "nFAULT")
    route(d, [(25.0, 13), (25.0, 15)], "gates")
    route(d, [(31.0, 15), (31.0, 13)])
    route(d, [(31.0, 9), (31.0, 7.0), (26.0, 7.0), (26.0, 6)], "I_A, I_B, I_C")
    route(d, [(34.3, 15), (34.3, 6)], "resolver,\nmotor NTC")
    route(d, [(36.0, 6), (36.0, 21.5), (10.0, 21.5), (10.0, 20)])
    d.add(elm.Label().at((18, 22.0)).label("contactor coils (12 V)", fontsize=9))
    route(d, [(1.0, 15), (1.0, -1.5), (33.0, -1.5), (33.0, 0)])
    d.add(elm.Label().at((16, -2.0)).label("CAN: BMS status and current limits", fontsize=9))
    route(d, [(10, 3), (12, 3)])
    d.add(elm.Label().at((11, 3.5)).label("rails", fontsize=9))
    route(d, [(29, 2), (27, 2)])
    route(d, [(27, 4), (29, 4)])
    d.add(elm.Line().at((-0.5, 7.8)).to((37.5, 7.8)).linestyle("--").color("#aa0000"))
    note(d, (2.2, 8.3), "HV / LV isolation boundary (reinforced)", 9)
    return d


# ==========================================================================
# Sheet 1: HV power path and power stage
# ==========================================================================
def sheet1():
    d = new_drawing()
    title(d, 0, 15, "Sheet 1  HV power path and power stage",
          f"DC link {farad(val('HV', 'c_dc'))} / {val('HV', 'c_dc_vrating'):.0f} V · precharge {ohm(val('HV', 'r_pre'))} · "
          f"active discharge {ohm(val('HV', 'r_dis'))} · bleeder {val('HV', 'n_bleed'):.0f} × {ohm(val('HV', 'r_bleed_each'))}")
    top, bot = 9.0, 0.0
    # battery
    d.add(elm.Battery().at((0, bot)).toy(top - 3).reverse().label("HV battery\n96s", loc="left"))
    wire(d, (0, top - 3), (0, top))
    d.add(elm.Fuse().at((0, top)).right().length(3).label("pack fuse", loc="top"))
    dot(d, (3, top))
    # K_POS and precharge branch
    d.add(elm.Switch().at((3, top)).right().tox(9).label("K_POS", loc="bottom"))
    wire(d, (3, top), (3, top + 2.5))
    d.add(elm.Switch().at((3, top + 2.5)).right().length(2.5).label("K_PRE"))
    d.add(elm.Resistor().right().tox(9).label(f"R_PRE\n{ohm(val('HV', 'r_pre'))}"))
    wire(d, (9, top + 2.5), (9, top))
    dot(d, (9, top))
    # K_NEG
    d.add(elm.Switch().at((0, bot)).right().tox(9).label("K_NEG", loc="bottom"))
    # rails
    wire(d, (9, top), (38.5, top))
    wire(d, (9, bot), (38.5, bot))
    note(d, (38.6, top), "DC+")
    note(d, (38.6, bot), "DC−")
    # DC link capacitor
    dot(d, (11, top)); dot(d, (11, bot))
    d.add(elm.Capacitor().at((11, top)).toy(bot).label(f"C_DC\n{farad(val('HV', 'c_dc'))}\n{val('HV', 'c_dc_vrating'):.0f} V", loc="bottom"))
    # bleeder
    dot(d, (13.5, top)); dot(d, (13.5, bot))
    d.add(elm.Resistor().at((13.5, top)).toy(bot).label(
        f"R_BLEED\n{val('HV', 'n_bleed'):.0f}×{ohm(val('HV', 'r_bleed_each'))}", loc="bottom"))
    # active discharge
    xd = 17.5
    dot(d, (xd, top)); dot(d, (xd, bot))
    d.add(elm.Resistor().at((xd, top)).toy(4.5).label(f"R_DIS\n{ohm(val('HV', 'r_dis'))}", loc="bottom"))
    q = d.add(elm.NFet().at((xd, 4.5)).anchor("drain").theta(0).reverse().label("Q_DIS", loc="right", ofst=(0.3, 0)))
    wire(d, q.source, (xd, bot))
    wire(d, q.gate, (q.gate[0] - 0.9, q.gate[1]))
    d.add(elm.Dot(open=True).at((q.gate[0] - 0.9, q.gate[1])))
    note(d, (q.gate[0] - 1.2, q.gate[1] - 1.1), "DISCHARGE drive\n(isolated, sheet 5)", 8)
    # DC voltage sense string
    dot(d, (21, top)); dot(d, (21, bot))
    d.add(elm.Resistor().at((21, top)).toy(3.2).label(
        f"R_VS1..5\n{val('VDC', 'n_top'):.0f}×{ohm(val('VDC', 'r_top_each'))}", loc="bottom"))
    dot(d, (21, 3.2))
    d.add(elm.Resistor().at((21, 3.2)).toy(bot).label(f"R_VS6\n{ohm(FW['VDC_DIV_R_BOT_OHM'])}", loc="bottom"))
    wire(d, (21, 3.2), (22.2, 3.2))
    net(d, (22.2, 3.2), "VDC_SENSE\n(sheet 3)")
    # half bridges
    for i, (ph, x) in enumerate((("A", 27), ("B", 31.5), ("C", 36))):
        dot(d, (x, top)); dot(d, (x, bot))
        hi = d.add(elm.NFet().at((x, top)).anchor("drain").theta(0).reverse().label(f"Q{2 * i + 1}", loc="right", ofst=(0.25, 0)))
        lo = d.add(elm.NFet().at((x, 4.5)).anchor("drain").theta(0).reverse().label(f"Q{2 * i + 2}", loc="right", ofst=(0.25, 0)))
        wire(d, hi.source, (x, 4.5))
        wire(d, lo.source, (x, bot))
        dot(d, (x, 4.5))
        wire(d, (x, 4.5), (x + 1.2, 4.5))
        net(d, (x + 1.2, 4.5), f"PH_{ph}")
        for g, n in ((hi.gate, f"G{ph}H"), (lo.gate, f"G{ph}L")):
            wire(d, g, (g[0] - 0.9, g[1]))
            d.add(elm.Dot(open=True).at((g[0] - 0.9, g[1])).label(n, loc="top", fontsize=8))
    d.add(Rect((24.4, -0.8), corner2=(38.2, 10.4)).linestyle("--").color("#888888"))
    note(d, (24.6, 10.8), f"PM1 IGBT six-pack {val('MODULE', 'v_ces'):.0f} V / {val('MODULE', 'i_c'):.0f} A "
         "(antiparallel diodes integrated)", 9)
    # motor section
    mot = d.add(Ic(size=(4, 5), pins=[
        elm.IcPin(name="U", side="L", slot="3/3"), elm.IcPin(name="V", side="L", slot="2/3"),
        elm.IcPin(name="W", side="L", slot="1/3"),
        elm.IcPin(name="R1/R2 EXC", side="R", slot="4/4", anchorname="EXC"),
        elm.IcPin(name="S1/S3 SIN", side="R", slot="3/4", anchorname="SIN"),
        elm.IcPin(name="S2/S4 COS", side="R", slot="2/4", anchorname="COS"),
        elm.IcPin(name="NTC", side="R", slot="1/4")]).theta(0).at((26, -3.0)).anchor("U").label("IPMSM\n+ resolver", loc="center"))
    for ph, pin in (("A", "U"), ("B", "V"), ("C", "W")):
        y = mot.absanchors[pin][1]
        net(d, (17, y), f"PH_{ph}", side="left")
        wire(d, (17, y), (19, y))
        d.add(Rect((19, y - 0.45), corner2=(21.6, y + 0.45), fill="#fff4e0"))
        note(d, (19.2, y), f"CS{'ABC'.index(ph) + 1} Hall", 8)
        wire(d, (21.6, y), mot.absanchors[pin])
    for p, n in (("EXC", "RES_EXC"), ("SIN", "RES_SIN"), ("COS", "RES_COS"), ("NTC", "NTC_MOT")):
        a = mot.absanchors[p]
        wire(d, a, (a[0] + 1.2, a[1]))
        net(d, (a[0] + 1.2, a[1]), n + " (sheet 3/5)" if n == "NTC_MOT" else n + " (sheet 5)")
    note(d, (0, -3.0), "Notes:", 10)
    note(d, (0, -3.8), "• Contactors, precharge relay and pack fuse are in the battery\n"
         "  junction box; coils driven from sheet 5.", 8)
    note(d, (0, -5.6), "• Phase current sensors are core-type Hall sensors around the\n"
         "  busbars: no galvanic connection, outputs on sheet 3.", 8)
    note(d, (0, -7.4), "• Gate and emitter connections of Q1…Q6: one driver channel\n"
         "  per switch, sheet 2.", 8)
    return d


# ==========================================================================
# Sheet 2: gate driver channel (one of six) and isolated bias supply
# ==========================================================================
def sheet2():
    d = new_drawing()
    title(d, 0, 17, "Sheet 2  Gate driver channel (1 of 6) and isolated bias supply",
          "Shown for the high-side switch of phase A; the other five channels are identical. "
          "Symbol pin order is functional, not package order.")
    u = d.add(Ic(size=(5, 7), pins=[
        elm.IcPin(name="IN+", side="L", slot="7/7", anchorname="INP"),
        elm.IcPin(name="IN−", side="L", slot="6/7", anchorname="INN"),
        elm.IcPin(name="RST/EN", side="L", slot="5/7", anchorname="EN"),
        elm.IcPin(name="FLT", side="L", slot="4/7", anchorname="FLT"),
        elm.IcPin(name="RDY", side="L", slot="3/7", anchorname="RDY"),
        elm.IcPin(name="VCC", side="L", slot="2/7", anchorname="VCC"),
        elm.IcPin(name="GND", side="L", slot="1/7", anchorname="GND"),
        elm.IcPin(name="DESAT", side="R", slot="7/7", anchorname="DESAT"),
        elm.IcPin(name="VDD", side="R", slot="6/7", anchorname="VDD"),
        elm.IcPin(name="OUTH", side="R", slot="5/7", anchorname="OUTH"),
        elm.IcPin(name="OUTL", side="R", slot="4/7", anchorname="OUTL"),
        elm.IcPin(name="CLMPI", side="R", slot="3/7", anchorname="CLMP"),
        elm.IcPin(name="COM", side="R", slot="2/7", anchorname="COM"),
        elm.IcPin(name="VEE", side="R", slot="1/7", anchorname="VEE"),
    ]).theta(0).at((8, 3)).label("U13  UCC21750-Q1", loc="top"))
    A = u.absanchors
    # primary side
    d.add(elm.Resistor().at(A["INP"]).left().length(2.5).label("100 Ω"))
    net(d, (A["INP"][0] - 2.5, A["INP"][1]), "PWM_AH\n(FlexPWM_0 A0)", side="left")
    wire(d, A["INN"], (A["INN"][0] - 0.8, A["INN"][1]))
    d.add(elm.Ground().at((A["INN"][0] - 0.8, A["INN"][1])))
    for pin, name in (("EN", "GATE_EN"), ("FLT", "nFAULT (wired-OR ×6)"), ("RDY", "nRDY (wired-OR ×6)")):
        wire(d, A[pin], (A[pin][0] - 2.5, A[pin][1]))
        net(d, (A[pin][0] - 2.5, A[pin][1]), name, side="left")
    wire(d, A["VCC"], (A["VCC"][0] - 1.5, A["VCC"][1]))
    d.add(elm.Vdd().at((A["VCC"][0] - 1.5, A["VCC"][1])).label("3V3", fontsize=9))
    wire(d, A["GND"], (A["GND"][0] - 0.8, A["GND"][1]))
    d.add(elm.Ground().at((A["GND"][0] - 0.8, A["GND"][1])))
    note(d, (-1.0, 0.6), "GATE_EN: 10 kΩ pull-down (sheet 4); low disables all six drivers.\n"
         "nFAULT: 4.7 kΩ pull-up to 3V3 → FlexPWM_0 FAULT0 and a GPIO.\n"
         "APWM/AIN (isolated analog channel) unused; option: module NTC.", 8)
    # isolation barrier (inside the driver package)
    xb = 10.5
    for y0, y1 in ((-0.6, 3.0), (10.0, 12.8)):
        d.add(elm.Line().at((xb, y0)).to((xb, y1)).linestyle("--").color("#aa0000"))
    note(d, (xb + 0.2, 12.4), "reinforced isolation", 8)
    # secondary side
    xs = A["VDD"][0]
    for pin, name in (("VDD", f"+{val('GATE', 'v_pos'):.0f}V_ISO_AH"), ("VEE", f"{val('GATE', 'v_neg'):.0f}V_ISO_AH")):
        wire(d, A[pin], (xs + 1.4, A[pin][1]))
        net(d, (xs + 1.4, A[pin][1]), name)
    gate_y = A["OUTL"][1]
    gate_x = xs + 8.0
    d.add(elm.Resistor().at(A["OUTH"]).right().length(3).label("R_GON 2.2 Ω", fontsize=9))
    wire(d, (xs + 3, A["OUTH"][1]), (xs + 3.6, A["OUTH"][1]), (xs + 3.6, gate_y))
    d.add(elm.Resistor().at(A["OUTL"]).right().length(3).label("R_GOFF 1 Ω", loc="bottom", fontsize=9))
    wire(d, (xs + 3, gate_y), (gate_x, gate_y))
    dot(d, (xs + 3.6, gate_y))
    wire(d, A["CLMP"], (xs + 5.0, A["CLMP"][1]), (xs + 5.0, gate_y))
    dot(d, (xs + 5.0, gate_y))
    q = d.add(elm.NFet().at((gate_x, gate_y)).anchor("gate").theta(0).reverse()
              .label("Q1 (PM1)", loc="right", ofst=(0.2, 0)))
    xc = q.drain[0]
    em_y = A["COM"][1]
    wire(d, q.source, (xc, em_y))
    dot(d, (xc, em_y))
    wire(d, (xc, em_y), (xc, -0.6))
    net(d, (xc, -0.6), "PH_A (emitter)", side="right")
    wire(d, A["COM"], (xc, em_y))
    # gate-emitter resistor
    xr = xs + 6.6
    dot(d, (xr, gate_y))
    d.add(elm.Resistor().at((xr, gate_y)).toy(em_y).label("R_GE\n10 kΩ", loc="bottom", fontsize=9))
    dot(d, (xr, em_y))
    # collector and DESAT
    yd = A["DESAT"][1]
    wire(d, q.drain, (xc, 12.0))
    net(d, (xc, 12.0), "DC+ (collector)", side="right")
    d.add(elm.Resistor().at(A["DESAT"]).right().length(3.0).label("1 kΩ", fontsize=9))
    d.add(elm.Diode().right().length(2.6).reverse().label("D_DESAT 2×1 kV", fontsize=9))
    wire(d, (xs + 5.6, yd), (xc, yd))
    dot(d, (xc, yd))
    dot(d, (xs + 0.8, yd))
    d.add(elm.Capacitor().at((xs + 0.8, yd)).up().length(1.3).label("C_BLK 33 pF → COM", loc="right", fontsize=8))

    # ---------------- isolated bias supply
    note(d, (0, -2.6), "Isolated bias supply (one per channel)", 11)
    sp = d.add(Ic(size=(3, 3), pins=[
        elm.IcPin(name="VCC", side="L", slot="3/3"), elm.IcPin(name="EN", side="L", slot="2/3"),
        elm.IcPin(name="GND", side="L", slot="1/3"),
        elm.IcPin(name="D1", side="R", slot="2/2"), elm.IcPin(name="D2", side="R", slot="1/2")]).theta(0).at((3, -8)).label(
        "U19  SN6505B-Q1", loc="top", fontsize=9))
    S = sp.absanchors
    for pin in ("VCC", "EN"):
        wire(d, S[pin], (S[pin][0] - 1, S[pin][1]))
        net(d, (S[pin][0] - 1, S[pin][1]), "5V_GD", side="left")
    wire(d, S["GND"], (S["GND"][0] - 0.6, S["GND"][1]))
    d.add(elm.Ground().at((S["GND"][0] - 0.6, S["GND"][1])))
    t = d.add(elm.Transformer(t1=4, t2=4, core=True).tap("pct", 2, "primary").tap("sct", 2, "secondary")
              .theta(0).at((S["D1"][0] + 2.0, S["D1"][1] + 0.4)).anchor("p1")
              .label("T1  reinforced", loc="top", fontsize=9))
    T = t.absanchors
    wire(d, S["D1"], (S["D1"][0] + 1.0, S["D1"][1]), (S["D1"][0] + 1.0, T["p1"][1]), T["p1"])
    wire(d, S["D2"], (S["D2"][0] + 1.0, S["D2"][1]), (S["D2"][0] + 1.0, T["p2"][1]), T["p2"])
    wire(d, T["pct"], (T["pct"][0] - 0.5, T["pct"][1]))
    d.add(elm.Dot(open=True).at((T["pct"][0] - 0.5, T["pct"][1])).label("5V_GD", loc="bottom", fontsize=8))
    xr2 = T["s1"][0] + 2.2
    y_top = T["s1"][1] + 1.0
    y_bot = T["s2"][1] - 1.6
    d.add(elm.Diode().at(T["s1"]).right().tox(xr2).label("D1", fontsize=9))
    d.add(elm.Diode().at(T["s2"]).right().tox(xr2).label("D2", loc="bottom", fontsize=9))
    wire(d, (xr2, T["s2"][1]), (xr2, y_top))
    dot(d, (xr2, T["s1"][1]))
    wire(d, T["sct"], (T["sct"][0] + 0.5, T["sct"][1]), (T["sct"][0] + 0.5, y_bot))
    # rails: VDD (top), COM (middle), VEE (bottom)
    x_end = xr2 + 8.5
    y_com = (y_top + y_bot) / 2 - 0.2
    wire(d, (xr2, y_top), (x_end, y_top))
    wire(d, (T["sct"][0] + 0.5, y_bot), (x_end, y_bot))
    net(d, (x_end, y_top), "VDD  +15 V (to U13 VDD)")
    net(d, (x_end, y_bot), "VEE  −5 V (to U13 VEE)")
    x1, x2, x3 = xr2 + 1.5, xr2 + 3.5, xr2 + 5.5
    dot(d, (x1, y_top)); dot(d, (x1, y_bot))
    d.add(elm.Capacitor().at((x1, y_top)).toy(y_com).label("10 µF", loc="bottom", fontsize=8))
    d.add(elm.Capacitor().at((x1, y_com)).toy(y_bot).label("10 µF", loc="bottom", fontsize=8))
    dot(d, (x2, y_top))
    d.add(elm.Resistor().at((x2, y_top)).toy(y_com).label("2.2 kΩ", loc="bottom", fontsize=8))
    dot(d, (x3, y_bot))
    d.add(elm.Zener().at((x3, y_bot)).toy(y_com).label("5.1 V", loc="bottom", fontsize=8))
    wire(d, (x1, y_com), (x_end, y_com))
    for x in (x1, x2, x3):
        dot(d, (x, y_com))
    net(d, (x_end, y_com), "COM  0 V = emitter")
    note(d, (xr2 - 0.5, y_bot - 1.2), "Rectified ≈ 20 V; zener holds COM 5.1 V above VEE → +15 V / −5 V gate drive.", 8)
    return d


# ==========================================================================
# Sheet 3: analog sensing
# ==========================================================================
def divider_to_adc(d, x, y, vin_net, r_top, r_bot, cap, adc_net, label_prefix):
    """Input net at (x,y) → series R → node → ADC; node → R → GND, node → C → GND."""
    if vin_net:
        net(d, (x, y), vin_net, side="left")
    d.add(elm.Resistor().at((x, y)).right().length(3).label(f"{label_prefix}1\n{ohm(r_top)}", fontsize=9))
    nx = x + 3
    dot(d, (nx, y))
    wire(d, (nx, y), (nx + 3.8, y))
    net(d, (nx + 3.8, y), adc_net)
    if r_bot:
        d.add(elm.Resistor().at((nx, y)).down().length(2.2).label(f"{label_prefix}2\n{ohm(r_bot)}", loc="bottom", fontsize=9))
        d.add(elm.Ground())
    dot(d, (nx + 1.8, y))
    d.add(elm.Capacitor().at((nx + 1.8, y)).down().length(2.2).label(farad(cap), loc="bottom", fontsize=9))
    d.add(elm.Ground())


def sheet3():
    d = new_drawing()
    title(d, 0, 22, "Sheet 3  Analog sensing",
          f"ADC reference VREFH = 3V3_A ({ADC_REF_TXT}). Values match config/board_cfg.h.")
    # --- phase currents
    note(d, (0, 19.5), "Phase current (×3, phase A shown)", 11)
    cs = d.add(Ic(size=(4.5, 2.4), pins=[
        elm.IcPin(name="+5V", side="L", slot="2/2", anchorname="VCC"),
        elm.IcPin(name="GND", side="L", slot="1/2", anchorname="GND"),
        elm.IcPin(name="OUT", side="R", slot="1/1", anchorname="OUT")]).at((2, 16)).label("CS1 Hall ±800 A", loc="top", fontsize=9))
    C = cs.absanchors
    wire(d, C["VCC"], (C["VCC"][0] - 0.8, C["VCC"][1]))
    net(d, (C["VCC"][0] - 0.8, C["VCC"][1]), "5V_PREC", side="left")
    wire(d, C["GND"], (C["GND"][0] - 0.8, C["GND"][1]))
    d.add(elm.Ground().at((C["GND"][0] - 0.8, C["GND"][1])))
    divider_to_adc(d, C["OUT"][0], C["OUT"][1], "", FW["CUR_DIV_R_TOP_OHM"], FW["CUR_DIV_R_BOT_OHM"],
                   val("CURRENT", "c_filt"), "IA → ADC_0 CH0", "R_IA")
    note(d, (10, 13.6), f"2.5 V ± 2.5 mV/A at the sensor → × {FW['CUR_DIV_R_BOT_OHM'] / (FW['CUR_DIV_R_TOP_OHM'] + FW['CUR_DIV_R_BOT_OHM']):.3f}\n"
         f"= {FW['CUR_A_PER_COUNT']:.3f} A/count, zero ≈ {FW['CUR_ZERO_COUNTS']:.0f} counts\n"
         "IB → ADC_1 CH0, IC → ADC_0 CH1 (identical circuits)", 8)

    # --- DC link voltage
    note(d, (0, 11.5), "DC link voltage (HV side left of barrier)", 11)
    u = d.add(Ic(size=(4, 4), pins=[
        elm.IcPin(name="VDD1", side="L", slot="4/4"), elm.IcPin(name="VINP", side="L", slot="3/4"),
        elm.IcPin(name="VINN", side="L", slot="2/4"), elm.IcPin(name="GND1", side="L", slot="1/4"),
        elm.IcPin(name="VDD2", side="R", slot="4/4"), elm.IcPin(name="VOUTP", side="R", slot="3/4"),
        elm.IcPin(name="VOUTN", side="R", slot="2/4"), elm.IcPin(name="GND2", side="R", slot="1/4")]
    ).at((5, 5)).label("U25\nAMC1311-Q1", loc="center"))
    A = u.absanchors
    net(d, (A["VINP"][0] - 3, A["VINP"][1]), "VDC_SENSE\n(sheet 1)", side="left")
    wire(d, (A["VINP"][0] - 3, A["VINP"][1]), A["VINP"])
    dot(d, (A["VINP"][0] - 1.2, A["VINP"][1]))
    d.add(elm.Capacitor().at((A["VINP"][0] - 1.2, A["VINP"][1])).toy(A["VINN"][1]).label("10 nF", loc="bottom", fontsize=8))
    wire(d, A["VINN"], (A["VINN"][0] - 3, A["VINN"][1]))
    net(d, (A["VINN"][0] - 3, A["VINN"][1]), "DC−", side="left")
    wire(d, A["GND1"], (A["GND1"][0] - 3, A["GND1"][1]))
    net(d, (A["GND1"][0] - 3, A["GND1"][1]), "DC−", side="left")
    wire(d, A["VDD1"], (A["VDD1"][0] - 3, A["VDD1"][1]))
    net(d, (A["VDD1"][0] - 3, A["VDD1"][1]), "5V_HV (LDO from\nphase A low-side\ngate supply, ref. DC−)", side="left")
    d.add(elm.Line().at((7, 3.2)).to((7, 10.2)).linestyle("--").color("#aa0000"))
    wire(d, A["VDD2"], (A["VDD2"][0] + 1, A["VDD2"][1]))
    net(d, (A["VDD2"][0] + 1, A["VDD2"][1]), "3V3")
    wire(d, A["GND2"], (A["GND2"][0] + 0.8, A["GND2"][1]))
    d.add(elm.Ground().at((A["GND2"][0] + 0.8, A["GND2"][1])))
    rf, rg = val("VDC", "opamp_rf"), val("VDC", "opamp_rg")
    # difference amplifier, flipped: + input on top
    op = d.add(elm.Opamp(leads=True).theta(0).flip().at((15.5, A["VOUTP"][1])).anchor("in2")
               .label("U26", loc="center", fontsize=9))
    d.add(elm.Resistor().at(A["VOUTP"]).right().length(4.0).label(f"R_G {ohm(rg)}", fontsize=9))
    wire(d, (A["VOUTP"][0] + 4.0, A["VOUTP"][1]), op.in2)
    xn = op.in1[0] - 1.0
    d.add(elm.Resistor().at(A["VOUTN"]).right().length(4.0).label(f"R_G {ohm(rg)}", loc="bottom", fontsize=9))
    wire(d, (A["VOUTN"][0] + 4.0, A["VOUTN"][1]), (xn, A["VOUTN"][1]), (xn, op.in1[1]), op.in1)
    dot(d, (xn, A["VOUTN"][1]))
    yfb = A["VOUTN"][1] - 1.6
    wire(d, (xn, A["VOUTN"][1]), (xn, yfb))
    d.add(elm.Resistor().at((xn, yfb)).right().tox(op.out[0] + 0.6).label(f"R_F {ohm(rf)}", loc="bottom", fontsize=9))
    wire(d, (op.out[0] + 0.6, yfb), (op.out[0] + 0.6, op.out[1]))
    xp = op.in2[0] - 0.6
    dot(d, (xp, op.in2[1]))
    d.add(elm.Resistor().at((xp, op.in2[1])).up().length(2.0).label(f"R_F {ohm(rf)}", loc="bottom", fontsize=9))
    d.add(elm.Dot(open=True).label("AGND", loc="top", fontsize=9))
    wire(d, op.out, (op.out[0] + 0.6, op.out[1]))
    dot(d, (op.out[0] + 0.6, op.out[1]))
    d.add(elm.Resistor().at((op.out[0] + 0.6, op.out[1])).right().length(2.4).label("100 Ω", fontsize=9))
    xo = op.out[0] + 3.0
    dot(d, (xo, op.out[1]))
    d.add(elm.Capacitor().at((xo, op.out[1])).down().length(2).label("10 nF", loc="bottom", fontsize=9))
    d.add(elm.Ground())
    wire(d, (xo, op.out[1]), (xo + 1.2, op.out[1]))
    net(d, (xo + 1.2, op.out[1]), "VDC → ADC_1 CH1")
    note(d, (18.5, 4.0), f"Gain {FW['VDC_DIV_R_BOT_OHM'] / (FW['VDC_DIV_R_TOP_OHM'] + FW['VDC_DIV_R_BOT_OHM']) * 1e3:.3f} mV/V × "
         f"{FW['VDC_ISO_GAIN']:g} × {rf / rg:g}\n= {FW['VDC_V_PER_COUNT']:.4f} V/count, "
         f"full scale {FW['VDC_V_PER_COUNT'] * 4095:.0f} V", 8)

    # --- NTC and pedal
    note(d, (0, 1.6), "Temperatures (×2)", 11)
    for i, (name, adc) in enumerate((("NTC_INV (cold plate)", "T_INV → ADC_2 CH2"), ("NTC_MOT (sheet 1)", "T_MOT → ADC_2 CH3"))):
        x0, y0 = 0.5 + 12 * i, -2.5
        d.add(elm.Vdd().at((x0 + 2, y0 + 2.4)).label("3V3_A", fontsize=9))
        d.add(elm.Resistor().at((x0 + 2, y0 + 2.4)).down().length(2.4).label(ohm(FW["NTC_PULLUP_OHM"]), loc="bottom", fontsize=9))
        dot(d, (x0 + 2, y0))
        d.add(elm.Resistor().at((x0 + 2, y0)).down().length(2.4).label(f"{name}\n10 kΩ B{FW['NTC_BETA']:.0f}", loc="bottom", fontsize=8))
        d.add(elm.Ground())
        d.add(elm.Resistor().at((x0 + 2, y0)).right().length(4.0).label("1 kΩ", fontsize=9))
        dot(d, (x0 + 6.0, y0))
        d.add(elm.Capacitor().at((x0 + 6.0, y0)).down().length(2).label("100 nF", loc="bottom", fontsize=9))
        d.add(elm.Ground())
        wire(d, (x0 + 6.0, y0), (x0 + 7.2, y0))
        net(d, (x0 + 7.2, y0), adc)

    note(d, (0, -7.5), "Accelerator pedal (two tracks, independent supplies)", 11)
    for i in range(2):
        x0, y0 = 2.5 + 15 * i, -10
        net(d, (x0, y0 + 1.6), f"5V_SENS{i + 1} → pedal track {i + 1} supply", side="right")
        divider_to_adc(d, x0, y0, f"APPS{i + 1}", FW["APPS_DIV_R_TOP_OHM"], FW["APPS_DIV_R_BOT_OHM"],
                       val("PEDAL", "c_filt"), f"APPS{i + 1} → ADC_2 CH{i}", f"R_P{i + 1}")
    return d


ADC_REF_TXT = f"±{val('ADC_REF', 'tol') * 100:g} %"


# ==========================================================================
# Sheet 4: MPC5744P connections
# ==========================================================================
def sheet4():
    d = new_drawing()
    title(d, 0, 19, "Sheet 4  MPC5744P signal assignment",
          "Signals by peripheral function. Package balls/pins: assign from the MPC5744P IO signal table; "
          "record in config/board_cfg.h.")
    left = [("PWM_AH", "FlexPWM_0 A0"), ("PWM_AL", "FlexPWM_0 B0"), ("PWM_BH", "FlexPWM_0 A1"),
            ("PWM_BL", "FlexPWM_0 B1"), ("PWM_CH", "FlexPWM_0 A2"), ("PWM_CL", "FlexPWM_0 B2"),
            ("nFAULT", "FlexPWM_0 FAULT0"), ("GATE_EN", "GPIO"), ("IA", "ADC_0 CH0"), ("IB", "ADC_1 CH0"),
            ("IC", "ADC_0 CH1"), ("VDC", "ADC_1 CH1"), ("APPS1", "ADC_2 CH0"), ("APPS2", "ADC_2 CH1"),
            ("T_INV", "ADC_2 CH2"), ("T_MOT", "ADC_2 CH3")]
    right = [("CAN_TX", "CAN_0 TXD"), ("CAN_RX", "CAN_0 RXD"), ("RDC_SCK", "DSPI_2 SCK"),
             ("RDC_SDO", "DSPI_2 SIN"), ("RDC_SDI", "DSPI_2 SOUT"), ("RDC_nCS", "DSPI_2 PCS0"),
             ("RDC_nSAMPLE", "GPIO out"), ("RDC_nFAULT", "GPIO in"), ("DI_KEY", "GPIO in"),
             ("DI_BRAKE", "GPIO in"), ("DI_P / R / N / D", "GPIO in ×4"), ("DO_K_NEG", "GPIO out"),
             ("DO_K_PRE", "GPIO out"), ("DO_K_POS", "GPIO out"), ("DO_DISCHARGE", "GPIO out"),
             ("CAN_nSTB", "GPIO out")]
    pins = []
    n = len(left) + 2
    for i, (nm, f) in enumerate(left):
        pins.append(elm.IcPin(name=f, side="L", slot=f"{n - 1 - i}/{n}", anchorname="L" + str(i)))
    for i, (nm, f) in enumerate(right):
        pins.append(elm.IcPin(name=f, side="R", slot=f"{n - 1 - i}/{n}", anchorname="R" + str(i)))
    pins += [elm.IcPin(name="EXTAL", side="B", slot="1/6"), elm.IcPin(name="XTAL", side="B", slot="2/6"),
             elm.IcPin(name="RESET", side="B", slot="3/6", anchorname="RST"),
             elm.IcPin(name="JTAG", side="B", slot="4/6", anchorname="JTAG"),
             elm.IcPin(name="VREFH", side="B", slot="5/6"), elm.IcPin(name="VDD", side="B", slot="6/6", anchorname="VDD")]
    u = d.add(Ic(size=(13, 18), pins=pins).at((10, 0)).label("U1\nMPC5744P", loc="center", fontsize=14))
    A = u.absanchors
    for i, (n, f) in enumerate(left):
        a = A["L" + str(i)]
        wire(d, a, (a[0] - 2.5, a[1]))
        net(d, (a[0] - 2.5, a[1]), n, side="left")
    for i, (n, f) in enumerate(right):
        a = A["R" + str(i)]
        wire(d, a, (a[0] + 2.5, a[1]))
        net(d, (a[0] + 2.5, a[1]), n, side="right")
    # crystal and bottom connections
    ex, xt = A["EXTAL"], A["XTAL"]
    wire(d, ex, (ex[0], ex[1] - 1.5))
    wire(d, xt, (xt[0], xt[1] - 1.5))
    d.add(elm.Crystal().at((ex[0], ex[1] - 1.5)).tox(xt[0]).label("Y1 40 MHz", loc="bottom", fontsize=9))
    for pin, name, ln in (("RST", "RESET_B\n(supervisor, JTAG)", 1.5), ("JTAG", "JTAG\n14-pin", 3.5),
                          ("VREFH", "3V3_A", 1.5), ("VDD", "3V3 + core\n(per data sheet)", 3.5)):
        a = A[pin]
        wire(d, a, (a[0], a[1] - ln))
        d.add(elm.Dot(open=True).at((a[0], a[1] - ln)).label(name, loc="bottom", fontsize=9))
    note(d, (-2, -8.0), "• Pull-down 10 kΩ on every PWM, GATE_EN and DO_* line: outputs are off while the MCU is in reset or unpowered.\n"
         "• nFAULT: 4.7 kΩ pull-up to 3V3, wired-OR of the six gate driver FLT outputs; also routed to a GPIO for diagnosis.\n"
         "• Decoupling: 100 nF per supply pin + bulk per data sheet; VREFH filtered from 3V3_A.", 8)
    return d


# ==========================================================================
# Sheet 5: vehicle interfaces
# ==========================================================================
def sheet5():
    d = new_drawing()
    title(d, 0, 23, "Sheet 5  Vehicle interfaces",
          "Digital inputs (×6), low-side outputs (×5), CAN, resolver-to-digital converter")
    # --- digital input
    note(d, (0, 20.5), "Digital input (×6: KEY, BRAKE, P, R, N, D), active high", 11)
    y = 17.5
    net(d, (0, y), "from switch\n(12 V)", side="left")
    wire(d, (0, y), (1.5, y))
    dot(d, (1.5, y))
    d.add(elm.Resistor().at((1.5, y)).down().length(2.4).label(f"R_WET\n{ohm(val('DIN', 'r_wet'))} 2512", loc="bottom", fontsize=9))
    d.add(elm.Ground())
    d.add(elm.Resistor().at((1.5, y)).right().length(3).label(ohm(val("DIN", "r_ser")), fontsize=9))
    dot(d, (4.5, y))
    d.add(elm.Resistor().at((4.5, y)).down().length(2.4).label(ohm(val("DIN", "r_sh")), loc="bottom", fontsize=9))
    d.add(elm.Ground())
    dot(d, (6.3, y))
    d.add(elm.Capacitor().at((6.3, y)).down().length(2.4).label(farad(val("DIN", "c")), loc="bottom", fontsize=9))
    d.add(elm.Ground())
    dot(d, (8.0, y))
    d.add(elm.Diode().at((8.0, y)).up().length(1.8).label("BAT54", loc="bottom", fontsize=8))
    d.add(elm.Vdd().label("3V3", fontsize=9))
    wire(d, (4.5, y), (10.5, y))
    net(d, (10.5, y), "DI_x (MCU)")

    # --- low side output
    note(d, (15, 20.5), "Low-side output (×5: K_NEG, K_PRE, K_POS, DISCHARGE*, spare)", 11)
    xl = 24
    net(d, (xl, 19.7), "12V_SW (fused)", side="right")
    d.add(elm.Inductor2().at((xl, 19.7)).down().length(2.1).label("contactor coil", loc="bottom", fontsize=9))
    q = d.add(elm.NFet().at((xl, 17.6)).anchor("drain").theta(0).reverse()
              .label("U28 protected low-side switch\n(integrated active drain clamp\n→ fast contactor release)",
                     loc="right", fontsize=8, ofst=(0.3, 0)))
    d.add(elm.Ground().at(q.source))
    d.add(elm.Resistor().at(q.gate).left().length(2.5).label("1 kΩ", fontsize=9))
    gx = q.gate[0] - 2.5
    dot(d, (gx, q.gate[1]))
    d.add(elm.Resistor().at((gx, q.gate[1])).down().length(1.8).label("10 kΩ", loc="bottom", fontsize=8))
    d.add(elm.Ground())
    wire(d, (gx, q.gate[1]), (gx - 1.2, q.gate[1]))
    net(d, (gx - 1.2, q.gate[1]), "DO_x", side="left")
    note(d, (15, 12.0), "*DISCHARGE drives the HV-side Q_DIS (sheet 1) through an isolated\n"
         " photovoltaic MOSFET driver instead of a coil.", 8)

    # --- CAN
    note(d, (0, 10.5), "CAN (500 kbit/s)", 11)
    c = d.add(Ic(size=(4, 5), pins=[
        elm.IcPin(name="TXD", side="L", slot="4/5"), elm.IcPin(name="RXD", side="L", slot="3/5"),
        elm.IcPin(name="STB", side="L", slot="2/5"), elm.IcPin(name="VIO", side="L", slot="1/5"),
        elm.IcPin(name="CANH", side="R", slot="4/5"), elm.IcPin(name="CANL", side="R", slot="1/5"),
        elm.IcPin(name="VCC", side="T", slot="1/1"), elm.IcPin(name="GND", side="B", slot="1/1")]
    ).at((3.5, 3)).label("U12\nTCAN1044AV-Q1", loc="center", fontsize=9))
    C = c.absanchors
    for p, n in (("TXD", "CAN_TX"), ("RXD", "CAN_RX"), ("STB", "CAN_nSTB"), ("VIO", "3V3")):
        wire(d, C[p], (C[p][0] - 1.5, C[p][1]))
        net(d, (C[p][0] - 1.5, C[p][1]), n, side="left")
    d.add(elm.Vdd().at(C["VCC"]).label("5V", fontsize=9))
    d.add(elm.Ground().at(C["GND"]))
    yh, yl = C["CANH"][1], C["CANL"][1]
    x0 = C["CANH"][0]
    d.add(Rect((x0 + 0.8, yl - 0.4), corner2=(x0 + 2.2, yh + 0.4)))
    note(d, (x0 + 0.85, yh + 0.9), "L_CM", 8)
    wire(d, C["CANH"], (x0 + 0.8, yh))
    wire(d, C["CANL"], (x0 + 0.8, yl))
    xt, xe = x0 + 4.0, x0 + 7.0
    wire(d, (x0 + 2.2, yh), (xe, yh))
    wire(d, (x0 + 2.2, yl), (xe, yl))
    net(d, (xe, yh), "CAN_H")
    net(d, (xe, yl), "CAN_L")
    dot(d, (xt, yh)); dot(d, (xt, yl))
    ym = (yh + yl) / 2
    d.add(elm.Resistor().at((xt, yh)).toy(ym).label("60 Ω", loc="bottom", fontsize=8))
    d.add(elm.Resistor().at((xt, ym)).toy(yl).label("60 Ω", loc="bottom", fontsize=8))
    dot(d, (xt, ym))
    d.add(elm.Capacitor().at((xt, ym)).left().length(1.2).label("4.7 nF", loc="bottom", fontsize=8))
    d.add(elm.Ground())
    note(d, (x0 + 1.0, yl - 1.4), "split termination: fit only at a bus end\nESD: ESD2CAN24-Q1 on CAN_H/CAN_L", 8)

    # --- RDC
    note(d, (15, 10.5), "Resolver-to-digital converter (excitation, SIN/COS to sheet 1)", 11)
    r = d.add(Ic(size=(6, 9), pins=[
        elm.IcPin(name="SCLK", side="L", slot="8/9"), elm.IcPin(name="SDO", side="L", slot="7/9"),
        elm.IcPin(name="SDI", side="L", slot="6/9"), elm.IcPin(name="WR/FSYNC", side="L", slot="5/9", anchorname="CS"),
        elm.IcPin(name="SAMPLE", side="L", slot="4/9"), elm.IcPin(name="LOT", side="L", slot="3/9"),
        elm.IcPin(name="DOS", side="L", slot="2/9"),
        elm.IcPin(name="EXC", side="R", slot="7/9", anchorname="EXC"),
        elm.IcPin(name="SIN", side="R", slot="5/9", anchorname="SIN"),
        elm.IcPin(name="COS", side="R", slot="4/9", anchorname="COS"),
        elm.IcPin(name="CLKIN", side="R", slot="2/9", anchorname="CLK"),
        elm.IcPin(name="VDRIVE", side="T", slot="1/2", anchorname="VDR"),
        elm.IcPin(name="VDD", side="T", slot="2/2", anchorname="VDD"),
        elm.IcPin(name="A0,A1,RES0,RES1", side="B", slot="1/1", anchorname="CFG")]).at((21, -1.5)).label("U9\nAD2S1210", loc="center", fontsize=10))
    R = r.absanchors
    for p, n in (("SCLK", "RDC_SCK"), ("SDO", "RDC_SDO"), ("SDI", "RDC_SDI"), ("CS", "RDC_nCS"), ("SAMPLE", "RDC_nSAMPLE")):
        wire(d, R[p], (R[p][0] - 1.6, R[p][1]))
        net(d, (R[p][0] - 1.6, R[p][1]), n, side="left")
    g = d.add(Ic(size=(1.2, 1.4), pins=[elm.IcPin(name="", side="R", slot="1/2", anchorname="A"),
                                            elm.IcPin(name="", side="R", slot="2/2", anchorname="B"),
                                            elm.IcPin(name="", side="L", slot="1/1", anchorname="Y")])
              .at((R["DOS"][0] - 2.0, R["DOS"][1] - 0.1)).anchor("A").label("AND", loc="center", fontsize=7))
    G = g.absanchors
    wire(d, R["LOT"], G["B"]) if abs(R["LOT"][1] - G["B"][1]) < 1e-6 else wire(d, R["LOT"], (G["B"][0], R["LOT"][1]), G["B"])
    wire(d, R["DOS"], G["A"]) if abs(R["DOS"][1] - G["A"][1]) < 1e-6 else wire(d, R["DOS"], (G["A"][0], R["DOS"][1]), G["A"])
    wire(d, G["Y"], (G["Y"][0] - 0.8, G["Y"][1]))
    net(d, (G["Y"][0] - 0.8, G["Y"][1]), "RDC_nFAULT", side="left")
    note(d, (G["Y"][0] - 0.6, G["Y"][1] - 1.1), "U11 74LVC1G08", 8)
    d.add(Rect((R["EXC"][0] + 0.6, R["EXC"][1] - 0.5), corner2=(R["EXC"][0] + 3.6, R["EXC"][1] + 0.5), fill="#fff4e0"))
    note(d, (R["EXC"][0] + 0.7, R["EXC"][1]), "U10 buffer", 8)
    wire(d, R["EXC"], (R["EXC"][0] + 0.6, R["EXC"][1]))
    wire(d, (R["EXC"][0] + 3.6, R["EXC"][1]), (R["EXC"][0] + 4.6, R["EXC"][1]))
    net(d, (R["EXC"][0] + 4.6, R["EXC"][1]), "RES_EXC")
    for p, n in (("SIN", "RES_SIN"), ("COS", "RES_COS")):
        wire(d, R[p], (R[p][0] + 4.6, R[p][1]))
        net(d, (R[p][0] + 4.6, R[p][1]), n + " (RC + ESD)")
    wire(d, R["CLK"], (R["CLK"][0] + 1.2, R["CLK"][1]))
    note(d, (R["CLK"][0] + 1.3, R["CLK"][1]), "Y2 8.192 MHz", 8)
    for pin, name in (("VDR", "3V3"), ("VDD", "5V")):
        wire(d, R[pin], (R[pin][0], R[pin][1] + 0.8))
        d.add(elm.Dot(open=True).at((R[pin][0], R[pin][1] + 0.8)).label(name, loc="top", fontsize=9))
    wire(d, R["CFG"], (R["CFG"][0], R["CFG"][1] - 0.8))
    note(d, (R["CFG"][0] - 3.2, R["CFG"][1] - 1.3), "strapped: normal mode, position, 16 bit (verify data sheet)", 8)
    return d


# ==========================================================================
# Sheet 6: LV power supply
# ==========================================================================
def reg(d, x, y, name, part, vout):
    u = d.add(Ic(size=(4.5, 2), pins=[elm.IcPin(name="IN", side="L", slot="1/1"),
                                      elm.IcPin(name="OUT", side="R", slot="1/1"),
                                      elm.IcPin(name="GND", side="B", slot="1/1")])
              .at((x, y)).anchor("IN").label(f"{name}  {part}", loc="top", fontsize=8))
    d.add(elm.Ground().at(u.absanchors["GND"]))
    o = u.absanchors["OUT"]
    wire(d, o, (o[0] + 1.6, o[1]))
    net(d, (o[0] + 1.6, o[1]), vout)
    return u


def sheet6():
    d = new_drawing()
    title(d, 0, 13, "Sheet 6  Low-voltage power supply",
          f"Battery {val('SUPPLY', 'vbat_min'):.0f}–{val('SUPPLY', 'vbat_max'):.0f} V continuous, "
          f"{val('SUPPLY', 'vbat_jump'):.0f} V jump start, {val('SUPPLY', 'vbat_rev'):.0f} V reverse polarity")
    y = 8
    net(d, (0, y), "KL30 (12 V)", side="left")
    d.add(elm.Fuse().at((0, y)).right().length(2.5).label("F1 5 A", fontsize=9))
    d.add(Rect((2.5, y - 0.7), corner2=(6.0, y + 0.7), fill="#eef4fb"))
    note(d, (2.6, y), "U27 LM74700-Q1\n+ N-FET (rev. pol.)", 8)
    wire(d, (6.0, y), (8.0, y))
    dot(d, (7.0, y))
    d.add(elm.Zener().at((7.0, y)).down().length(2.2).reverse().label("TVS", loc="bottom", fontsize=9))
    d.add(elm.Ground())
    d.add(elm.Inductor2().at((8.0, y)).right().length(2.4).label("π filter", fontsize=9))
    dot(d, (10.4, y))
    d.add(elm.Capacitor().at((10.4, y)).down().length(2.2).label("47 µF", loc="bottom", fontsize=9))
    d.add(elm.Ground())
    xb = 12.5
    wire(d, (10.4, y), (14, y))
    dot(d, (xb, y))
    wire(d, (xb, y), (xb, y + 1.6))
    net(d, (xb, y + 1.6), "12V_SW (contactor drivers, sheet 5)", side="right")
    reg(d, 14, y, "U2 buck", f"5 V / {val('LV', 'buck5_rating'):.0f} A", "5V")
    wire(d, (xb, y), (xb, 4.5), (14, 4.5))
    reg(d, 14, 4.5, "U3 buck", f"5 V / {val('LV', 'buck5gd_rating'):.0f} A", "5V_GD (isolated gate supplies)")
    note(d, (0, 4.6), "U3 feeds the six isolated gate supplies (sheet 2),\nkeeping their switching noise off the logic rail.", 8)
    # from 5V
    regs = (("U4 LDO", f"3.3 V / {val('LV', 'ldo33_rating'):.0f} A", "3V3 (MCU, logic)"),
            ("U5 LDO", f"3.3 V ±{val('ADC_REF', 'tol') * 100:g} %", "3V3_A (VREFH, NTC pull-ups)"),
            ("U6 prec.", f"5.000 V ±{val('CURRENT', 'supply_tol') * 100:g} %", "5V_PREC (current sensors)"),
            ("U7 tracking", "5 V, protected", "5V_SENS1 (pedal track 1)"),
            ("U8 tracking", "5 V, protected", "5V_SENS2 (pedal track 2)"))
    y0, dy = 0.5, 3.8
    yl = y0 - dy * (len(regs) - 1)
    net(d, (12.5, y0 + 1.5), "5V", side="left")
    wire(d, (12.5, y0 + 1.5), (13.0, y0 + 1.5), (13.0, yl))
    for k, (name, part, out) in enumerate(regs):
        yy = y0 - dy * k
        dot(d, (13.0, yy)) if k < len(regs) - 1 else None
        wire(d, (13.0, yy), (14.0, yy))
        reg(d, 14, yy, name, part, out)
    note(d, (0, -1.5), "MCU core supply per MPC5744P data sheet\n(internal regulator with external ballast,\n"
         "or external 1.25 V). A power-on reset\nsupervisor on 3V3 drives RESET_B.", 8)
    note(d, (0, -7.0), "To verify in design review: TVS stand-off ≥ 26 V\n(no conduction during jump start) and clamping\n"
         "voltage below the buck converters' maximum input.", 8)
    return d


SHEETS = [("sheet0_block_diagram", sheet0), ("sheet1_hv_power_stage", sheet1),
          ("sheet2_gate_driver", sheet2), ("sheet3_analog_sensing", sheet3),
          ("sheet4_mcu", sheet4), ("sheet5_interfaces", sheet5), ("sheet6_power_supply", sheet6)]


def write_bom():
    path = os.path.join(OUT, "bom.md")
    lines = ["# Concept bill of materials", "",
             "> Generated by `make schematics` from `tools/hw_design.py`. Status: **selected** = part chosen; "
             "**verify** = candidate part, data sheet values used in the analysis must be confirmed; "
             "**open** = only the requirement is fixed.", "",
             "| Ref | Function | Candidate part | Status |", "|---|---|---|---|"]
    for ref, fn, part, status in D.BOM:
        lines.append(f"| {ref} | {fn} | {part} | {status} |")
    lines.append("")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print("written", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", help="also write PNG previews to this directory")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for name, fn in SHEETS:
        schemdraw.use("svg")
        d = fn()
        d.save(os.path.join(OUT, name + ".svg"))
        if a.png:
            schemdraw.use("matplotlib")
            d = fn()
            d.save(os.path.join(a.png, name + ".png"), dpi=110)
        print("drawn", name)
    write_bom()


if __name__ == "__main__":
    main()
