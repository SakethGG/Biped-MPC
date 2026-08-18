"""One-off sanity check for the lambda-derived link mass model (see
builder.py's _linear_density). Builds the current design, loads it into
MuJoCo to read back each body's actual (auto-computed) mass, and renders a
PNG table comparing that against the expected M = lambda * L for the
thigh/shank/foot links. Throwaway script -- not imported by anything else.
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
XML_BUILDER_DIR = HERE.parent   # builder.py/generate.py live one level up, in graphs/'s parent
sys.path.insert(0, str(XML_BUILDER_DIR))

import mujoco
from PIL import Image, ImageDraw, ImageFont

import builder
from builder import build_xml, LAMBDA_THIGH, LAMBDA_SHANK, LAMBDA_FOOT
from generate import load_design_spec

spec = load_design_spec()
xml_str = build_xml(spec)
model = mujoco.MjModel.from_xml_string(xml_str)

# body name -> (lambda used, link's real full length) for the three
# lambda-derived links -- None for bodies whose mass doesn't come from the
# linear mass model (actuator housings use catalog mass, the trunk uses its
# own fixed density). Full length, NOT the LINK_BOX_LENGTH_FRAC-shortened
# box geom length -- thigh/shank boxes are drawn short purely so adjacent
# links don't visually overlap at the joint; builder.py's THIGH_DENSITY/
# SHANK_DENSITY already compensate for that so mass = lambda * full_length.
LINEAR_MODEL_INPUTS = {}
for side in builder.LEG_SIDES:
    LINEAR_MODEL_INPUTS[f"thigh_{side}_Link"] = (LAMBDA_THIGH, spec.thigh_length)
    LINEAR_MODEL_INPUTS[f"shank_{side}_Link"] = (LAMBDA_SHANK, spec.shank_length)
    LINEAR_MODEL_INPUTS[f"foot_{side}_Link"] = (LAMBDA_FOOT, spec.foot_length)

ROW_ORDER = ["base_Link"]
for side in builder.LEG_SIDES:
    ROW_ORDER += [
        f"abad_{side}_act", f"hip_{side}_act", f"knee_{side}_act",
        f"thigh_{side}_Link", f"shank_{side}_Link",
        f"ankle_{side}_act", f"foot_{side}_Link",
    ]

rows = []  # (body, lambda_str, L_used_str, expected_str, actual_str)
total_mass = 0.0
for name in ROW_ORDER:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    actual = float(model.body_mass[body_id])
    total_mass += actual
    if name in LINEAR_MODEL_INPUTS:
        lam, length = LINEAR_MODEL_INPUTS[name]
        expected = lam * length
        rows.append((name, f"{lam:.4f}", f"{length:.4f}", f"{expected:.6f}", f"{actual:.6f}"))
    else:
        rows.append((name, "—", "—", "—", f"{actual:.6f}"))

rows.append(("TOTAL (robot)", "", "", "", f"{total_mass:.6f}"))

for r in rows:
    print(f"{r[0]:<16} lambda={r[1]:<8} L_used={r[2]:<9} expected={r[3]:<10} actual={r[4]}")

# ---------------------------------------------------------------------------
# Render as a PNG table.
# ---------------------------------------------------------------------------
HEADERS = ["Body", "lambda (kg/m)", "L used (m)", "Expected M=lambda*L (kg)", "Actual mass (kg)"]
ROW_H = 30
HEADER_H = 36
PAD = 14
CELL_PAD = 14
FONT_SIZE = 15

try:
    font = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", FONT_SIZE)
    font_bold = ImageFont.truetype("/System/Library/Fonts/Menlo.ttc", FONT_SIZE)
except OSError:
    font = ImageFont.load_default(size=FONT_SIZE)
    font_bold = font

_meas = ImageDraw.Draw(Image.new("RGB", (1, 1)))


def _text_w(s: str, f) -> int:
    return int(_meas.textbbox((0, 0), s, font=f)[2])


COL_W = []
for col_idx, header in enumerate(HEADERS):
    widest = _text_w(header, font_bold)
    for row in rows:
        widest = max(widest, _text_w(row[col_idx], font_bold))
    COL_W.append(widest + 2 * CELL_PAD)

W = 2 * PAD + sum(COL_W)
H = 2 * PAD + HEADER_H + ROW_H * len(rows) + 40

img = Image.new("RGB", (W, H), "white")
draw = ImageDraw.Draw(img)

draw.text((PAD, PAD - 4), "Link mass sanity check (lambda-derived density model)", font=font_bold, fill="black")
y0 = PAD + 22

x = PAD
for header, w in zip(HEADERS, COL_W):
    draw.rectangle([x, y0, x + w, y0 + HEADER_H], fill=(40, 40, 40))
    draw.text((x + CELL_PAD, y0 + 9), header, font=font_bold, fill="white")
    x += w

y = y0 + HEADER_H
for i, row in enumerate(rows):
    is_total = row[0].startswith("TOTAL")
    bg = (255, 240, 200) if is_total else ((245, 245, 245) if i % 2 else (255, 255, 255))
    draw.rectangle([PAD, y, PAD + sum(COL_W), y + ROW_H], fill=bg)
    x = PAD
    for val, w in zip(row, COL_W):
        draw.text((x + CELL_PAD, y + 7), val, font=font_bold if is_total else font, fill="black")
        x += w
    y += ROW_H

draw.rectangle([PAD, y0, PAD + sum(COL_W), y], outline=(0, 0, 0), width=1)

out_path = HERE / "mass_report.png"
img.save(out_path)
print(f"\nWrote {out_path}")
