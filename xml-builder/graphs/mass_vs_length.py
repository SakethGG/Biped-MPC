"""Tracks Mass vs Length for the thigh/shank links across design iterations.

Each run reads the CURRENT thigh_length/shank_length out of
variable_params.json, builds the design, and reads back the actual
MuJoCo-computed mass for thigh_R_Link/shank_R_Link. If that length hasn't
been recorded before, it's appended to mass_vs_length_history.json; if it's
already there (i.e. you re-ran without changing the length), nothing is
appended. The plot is then redrawn from the full accumulated history, so it
grows a new point every time you change thigh_length/shank_length and re-run.

Run with the project's .venv (needs mujoco + matplotlib):
    ../../.venv/bin/python3 mass_vs_length.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
XML_BUILDER_DIR = HERE.parent   # builder.py/generate.py live one level up, in graphs/'s parent
sys.path.insert(0, str(XML_BUILDER_DIR))

import matplotlib.pyplot as plt
import mujoco

from builder import build_xml
from generate import load_design_spec

HISTORY_PATH = HERE / "mass_vs_length_history.json"
PLOT_PATH = HERE / "mass_vs_length.png"

LINKS = {
    "thigh": {"length_attr": "thigh_length", "body": "thigh_R_Link", "color": "tab:blue", "label": "Thigh Mass"},
    "shank": {"length_attr": "shank_length", "body": "shank_R_Link", "color": "tab:green", "label": "Shank Mass"},
}


def load_history() -> dict:
    if HISTORY_PATH.exists():
        with open(HISTORY_PATH) as f:
            return json.load(f)
    return {"thigh": [], "shank": []}


def save_history(history: dict) -> None:
    with open(HISTORY_PATH, "w") as f:
        json.dump(history, f, indent=2)


spec = load_design_spec()
xml_str = build_xml(spec)
model = mujoco.MjModel.from_xml_string(xml_str)

history = load_history()

for key, info in LINKS.items():
    length = getattr(spec, info["length_attr"])
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, info["body"])
    mass = float(model.body_mass[body_id])

    already_recorded = any(abs(pt["length"] - length) < 1e-9 for pt in history[key])
    if already_recorded:
        print(f"{key}: length {length:.4f} m already recorded, not appending")
    else:
        history[key].append({"length": length, "mass": mass})
        print(f"{key}: appended length={length:.4f} m, mass={mass:.6f} kg")

save_history(history)

# ---------------------------------------------------------------------------
# Plot -- one subplot per link, points sorted by length, redrawn from the
# full accumulated history every run.
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(9, 4))

for ax, (key, info) in zip(axes, LINKS.items()):
    pts = sorted(history[key], key=lambda p: p["length"])
    xs = [p["length"] for p in pts]
    ys = [p["mass"] for p in pts]
    ax.plot(xs, ys, color=info["color"], label=info["label"], marker="o" if len(xs) < 15 else None)
    ax.set_title(f"Mass vs {key.capitalize()} Length")
    ax.set_xlabel("Link Length (m)")
    ax.set_ylabel("Mass (kg)")
    ax.grid(True, alpha=0.6)
    ax.legend()

fig.tight_layout()
fig.savefig(PLOT_PATH, dpi=150)
print(f"Wrote {PLOT_PATH}")
