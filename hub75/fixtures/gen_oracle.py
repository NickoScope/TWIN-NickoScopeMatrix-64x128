#!/usr/bin/env python3
"""Generate hub75/tests/oracle/mod.rs from the Python oracle.

The oracle is the research re-implementation of ESP32-HUB75-MatrixPanel-DMA 3.0.14's
buffer build and a reference stream decoder:
    tools/twin/research/model.py in NickoScope/AnimatedPixelClock (HUB75_ORACLE_DIR)
It reads lumConvTab_8bit from the library header that the firmware compiles
(HUB75_CIE_LUTS), in the firmware checkout after a PlatformIO build:
    .pio/libdeps/matrix-waveshare-rgb/ESP32 HUB75 LED MATRIX PANEL DMA Display/src/cie_luts.h

Run (takes about two minutes, the oracle decoder is plain Python):
    python3 hub75/fixtures/gen_oracle.py hub75/tests/oracle/mod.rs

Nothing here decides a number on its own: every buffer is built by model.py's
clear_fb / set_brightness / paint, and every expected on-clock count comes from
model.decode's state machine. decode_passes() below is model.decode's loop, line
for line, run over an explicit sequence of passes so a double-buffer flip can be
expressed; see the comment on it.
"""
import os
import re
import sys

import os

# Where the oracle and the library header are on this machine (both from the firmware repository).
ORACLE_DIR = os.environ.get("HUB75_ORACLE_DIR") or sys.exit("set HUB75_ORACLE_DIR to the firmware repo's tools/twin/research")
LUT_PATH = os.environ.get("HUB75_CIE_LUTS") or sys.exit("set HUB75_CIE_LUTS to the HUB75 library's src/cie_luts.h (firmware .pio/libdeps)")

sys.path.insert(0, ORACLE_DIR)
_stdout = sys.stdout
sys.stdout = open(os.devnull, "w")  # model.py prints its t-loop at import
import model  # noqa: E402
sys.stdout.close()
sys.stdout = _stdout

W, H, RPF, DEPTH = model.W, model.H, model.RPF, model.DEPTH
src = open(LUT_PATH).read()
LUT = [int(x) for x in re.search(r"lumConvTab_8bit\[256\] = \{(.*?)\};", src, re.S)
       .group(1).replace("\n", " ").split(",") if x.strip()]
assert len(LUT) == 256


def fnv1a64(data: bytes) -> int:
    h = 0xcbf29ce484222325
    for b in data:
        h ^= b
        h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def stream_of(fb):
    s = []
    for r in range(RPF):
        for p in model.seq:
            s += fb[r][p]
    return s


def words_fnv(words):
    return fnv1a64(b"".join(w.to_bytes(2, "little") for w in words))


def acc_fnv(acc):
    return fnv1a64(b"".join(acc[y][x][c].to_bytes(4, "little")
                            for y in range(H) for x in range(W) for c in range(3)))


def build(brt, paints):
    fb = model.clear_fb()
    model.set_brightness(fb, brt)
    for (x, y, r, g, b) in paints:
        model.paint(fb, x, y, r, g, b, LUT)
    return fb


def decode_passes(streams):
    """model.decode's loop (model.py:52-73) over a sequence of passes.

    model.decode(fb, frames=2) feeds the same pass twice and measures the second.
    Here each pass may come from a different buffer (a flip), and every pass after the
    first is measured. The per-word statements are model.decode's, unchanged.
    """
    shift = [0] * W
    latched = [0] * W
    out = []
    for f, stream in enumerate(streams):
        acc = [[[0, 0, 0] for _ in range(W)] for _ in range(H)]
        total = 0
        for w in stream:
            shift.pop(0); shift.append(w & 0x3F)
            if w & model.BIT_LAT:
                latched = shift[:]
            total += 1
            if not (w & model.BIT_OE):
                a = (w >> model.ADDR) & 0x1F
                for x in range(W):
                    v = latched[x]
                    for ch in range(3):
                        if v >> ch & 1: acc[a][x][ch] += 1
                        if v >> (3 + ch) & 1: acc[a + RPF][x][ch] += 1
        if f > 0:
            out.append((acc, total))
    return out


def lit_of(acc):
    return [(y, x, tuple(acc[y][x])) for y in range(H) for x in range(W) if any(acc[y][x])]


def all_pixels(r, g, b):
    return [(x, y, r, g, b) for y in range(H) for x in range(W)]


# ---------------------------------------------------------------- scenarios
POSITIONS = [  # check2.py:6-7, plus the two remaining corners and the panel seam
    (10, 10, 255, 0, 0), (20, 42, 0, 255, 0), (0, 0, 0, 0, 255), (127, 63, 255, 255, 255),
    (5, 31, 255, 0, 0), (6, 32, 0, 0, 255),
    (127, 0, 255, 0, 0), (0, 63, 0, 255, 0), (63, 31, 255, 255, 255), (64, 32, 255, 255, 255),
]
RAMP = [(i, 3, v, 0, 0) for i, v in enumerate(range(186, 201))]  # check2.py:12-13
GRADIENT = ([(x, 5, x, 0, 0) for x in range(W)] + [(x, 40, x + 128, 0, 0) for x in range(W)]
            + [(0, 0, 255, 255, 255)])  # model.py:91-94

SCENARIOS = [
    # name, brightness, fill (None or rgb), paints
    ("positions_255", 255, None, POSITIONS),
    ("ramp_186_200_255", 255, None, RAMP),
    ("ramp_186_200_50", 50, None, RAMP),
    ("gradient_255", 255, None, GRADIENT),
    ("gradient_50", 50, None, GRADIENT),
    ("white_255", 255, (255, 255, 255), []),
    ("white_50", 50, (255, 255, 255), []),
]

FLIP_A = [(10, 31, 255, 0, 0), (10, 63, 0, 255, 0), (100, 5, 0, 0, 255)]
FLIP_B = [(20, 31, 255, 255, 255), (11, 0, 255, 0, 0)]


def rs_paints(p):
    return "&[" + ", ".join("(%d, %d, %d, %d, %d)" % t for t in p) + "]"


def rs_lit(lit):
    return "&[" + ", ".join("(%d, %d, [%d, %d, %d])" % (y, x, *v) for (y, x, v) in lit) + "]"


def main(out_path):
    o = []
    o.append("// @generated by hub75/fixtures/gen_oracle.py from the Python oracle")
    o.append("// (AnimatedPixelClock-twin tools/twin/research/model.py). Do not edit by hand.")
    o.append("// Regenerate: python3 hub75/fixtures/gen_oracle.py hub75/tests/oracle/mod.rs")
    o.append("#![allow(dead_code, clippy::type_complexity)]")
    o.append("")
    o.append("pub const T: usize = %d; // lsbMsbTransitionBit from model.py:7-15" % model.t)
    o.append("pub const CALC_REFRESH_HZ: u32 = %d; // library formula at i2sspeed 8 MHz" % model.calc_rr)
    o.append("pub const SEQ: [u8; %d] = [%s];" % (len(model.seq), ", ".join(map(str, model.seq))))
    o.append("/// lumConvTab_8bit parsed from cie_luts.h by the oracle's own regex.")
    o.append("pub const LUT: [u8; 256] = [%s];" % ", ".join(map(str, LUT)))

    oe = []
    for brt in [255, 128, 50, 10, 1, 0]:
        fb = model.clear_fb(); model.set_brightness(fb, brt)
        oe.append((brt, model.oe_counts(fb)))
    o.append("/// (brightness, OE-low words per plane memory 0..7), model.oe_counts.")
    o.append("pub const OE_COUNTS: &[(u8, [u32; 8])] = &[%s];" % ", ".join(
        "(%d, [%s])" % (b, ", ".join(map(str, c))) for b, c in oe))

    # per-plane on-clocks: model.py:99-108 (red bit of plane p at row 10, x 10)
    pw = []
    for brt in [255, 50]:
        ws = []
        for p in range(DEPTH):
            fb2 = model.clear_fb(); model.set_brightness(fb2, brt)
            fb2[10][p][10] |= 1
            a, _ = model.decode(fb2)
            ws.append(a[10][10][0])
            assert sum(1 for y in range(H) for x in range(W) if any(a[y][x])) == 1
        pw.append((brt, ws))
        print("plane weights brt", brt, ws, file=sys.stderr)
    o.append("/// (brightness, on-clocks of a single set bit in plane p at pixel (10,10) red),")
    o.append("/// model.py:99-108 decode.")
    o.append("pub const PLANE_WEIGHTS: &[(u8, [u32; 8])] = &[%s];" % ", ".join(
        "(%d, [%s])" % (b, ", ".join(map(str, w))) for b, w in pw))

    o.append("")
    o.append("pub struct Scenario {")
    o.append("    pub name: &'static str,")
    o.append("    pub brightness: u8,")
    o.append("    /// Painted pixel by pixel before `paints` (the oracle has no fill call).")
    o.append("    pub fill: Option<[u8; 3]>,")
    o.append("    /// (x, y, r, g, b) drawPixel calls, applied in order.")
    o.append("    pub paints: &'static [(u16, u16, u8, u8, u8)],")
    o.append("    pub stream_len: usize,")
    o.append("    /// FNV-1a 64 of one chain pass, u16 little-endian.")
    o.append("    pub stream_fnv: u64,")
    o.append("    /// FNV-1a 64 of on_clk[y][x][c] as u32 little-endian, second of two passes.")
    o.append("    pub onclk_fnv: u64,")
    o.append("    pub total: u64,")
    o.append("    pub lit_count: usize,")
    o.append("    /// (y, x, [r, g, b] on-clocks) of every lit LED, empty when lit_count > 400.")
    o.append("    pub lit: &'static [(u16, u16, [u32; 3])],")
    o.append("}")
    o.append("")
    o.append("pub const SCENARIOS: &[Scenario] = &[")
    for name, brt, fill, paints in SCENARIOS:
        full = (all_pixels(*fill) if fill else []) + paints
        fb = build(brt, full)
        s = stream_of(fb)
        acc, total = model.decode(fb)
        lit = lit_of(acc)
        print(name, "words", len(s), "total", total, "lit", len(lit), file=sys.stderr)
        o.append("    Scenario {")
        o.append('        name: "%s",' % name)
        o.append("        brightness: %d," % brt)
        o.append("        fill: %s," % ("Some([%d, %d, %d])" % fill if fill else "None"))
        o.append("        paints: %s," % rs_paints(paints))
        o.append("        stream_len: %d," % len(s))
        o.append("        stream_fnv: 0x%016x," % words_fnv(s))
        o.append("        onclk_fnv: 0x%016x," % acc_fnv(acc))
        o.append("        total: %d," % total)
        o.append("        lit_count: %d," % len(lit))
        o.append("        lit: %s," % (rs_lit(lit) if len(lit) <= 400 else "&[]"))
        o.append("    },")
    o.append("];")

    # double buffer: passes A, A, B, B (flip lands at the chain wrap after the 2nd A)
    fa = build(255, FLIP_A); fbb = build(255, FLIP_B)
    sa, sb = stream_of(fa), stream_of(fbb)
    res = decode_passes([sa, sa, sb, sb])
    o.append("")
    o.append("/// Double buffer: chain A shown twice, flip, chain B twice (brightness 255).")
    o.append("/// REFRESHES[i] is the pass i+1 decode: steady A, first B after the flip, steady B.")
    o.append("pub const FLIP_A: &[(u16, u16, u8, u8, u8)] = %s;" % rs_paints(FLIP_A))
    o.append("pub const FLIP_B: &[(u16, u16, u8, u8, u8)] = %s;" % rs_paints(FLIP_B))
    o.append("pub const FLIP_A_STREAM_FNV: u64 = 0x%016x;" % words_fnv(sa))
    o.append("pub const FLIP_B_STREAM_FNV: u64 = 0x%016x;" % words_fnv(sb))
    o.append("pub const FLIP_REFRESHES: &[(u64, &[(u16, u16, [u32; 3])])] = &[")
    for acc, total in res:
        lit = lit_of(acc)
        print("flip pass total", total, "lit", lit, file=sys.stderr)
        o.append("    (%d, %s)," % (total, rs_lit(lit)))
    o.append("];")
    o.append("")
    with open(out_path, "w") as f:
        f.write("\n".join(o))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "hub75/tests/oracle/mod.rs")
