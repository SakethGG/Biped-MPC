"""Plot torque vs. angular velocity (phase portrait) for every joint, with a
mass/length summary box for each link.

Reads qd/tau time series from the CSV written by SimulationRunner::
writeStateTelemetry(), and mass/link-length from the MJCF model (via
plot_robot_properties.collect_links) for the annotation box.

Usage:
    python3 plots/plot_torque_velocity.py
    python3 plots/plot_torque_velocity.py --csv logs/debug/state_telemetry/state_telemetry.csv --show
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_csv import default_state_telemetry_path, read_columns  # noqa: E402
from plot_robot_properties import collect_links  # noqa: E402

_JOINT_ORDER = ["abad", "hip", "knee", "ankle"]
_LEGS = ["right", "left"]


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=default_state_telemetry_path())
    parser.add_argument("--xml", type=Path, default=repo_root / "models" / "tron" / "biped.xml")
    parser.add_argument(
        "--out", type=Path, default=Path(__file__).resolve().parent / "torque_velocity.png"
    )
    parser.add_argument("--show", action="store_true", help="Also display the plot interactively")
    parser.add_argument("--figwidth", type=float, default=16.0)
    parser.add_argument("--figheight", type=float, default=9.0)
    return parser.parse_args()


def joint_properties(xml_path: Path) -> Dict[Tuple[str, str], Tuple[float, float]]:
    """(leg, joint) -> (mass_kg, link_length_m), e.g. ("right", "abad") -> (0.6, 0.207)."""
    root = ET.parse(xml_path).getroot()
    props: Dict[Tuple[str, str], Tuple[float, float]] = {}
    for link in collect_links(root):
        if link.joint_name is None:
            continue
        parts = link.joint_name.split("_")  # e.g. ["abad", "R", "Joint"]
        if len(parts) < 2:
            continue
        joint = parts[0].lower()
        side = "right" if parts[1].upper() == "R" else "left"
        props[(side, joint)] = (link.mass or 0.0, link.link_length)
    return props


def main() -> None:
    args = parse_args()
    columns = read_columns(args.csv)
    props = joint_properties(args.xml)

    fig, axes = plt.subplots(
        len(_LEGS), len(_JOINT_ORDER), figsize=(args.figwidth, args.figheight)
    )
    fig.suptitle("Torque vs Angular Velocity")

    for row, leg in enumerate(_LEGS):
        for col, joint in enumerate(_JOINT_ORDER):
            ax = axes[row][col]
            qd_key = f"{leg}_{joint}_qd"
            tau_key = f"{leg}_{joint}_tau"
            if qd_key in columns and tau_key in columns:
                ax.scatter(columns[qd_key], columns[tau_key], s=3, alpha=0.35, color="tab:blue")
            ax.set_title(f"{leg.capitalize()} {joint.upper()}")
            ax.set_xlabel("Angular velocity (rad/s)")
            ax.set_ylabel("Torque (Nm)")
            ax.grid(True, alpha=0.3)

    lines = []
    for leg in _LEGS:
        lines.append(f"{leg.capitalize()} leg:")
        for joint in _JOINT_ORDER:
            mass, length = props.get((leg, joint), (0.0, 0.0))
            lines.append(f"  {joint.upper()}: mass={mass:.3f} kg, length={length:.3f} m")
    summary_text = "\n".join(lines)

    fig.tight_layout(rect=(0, 0.22, 1, 0.95))
    fig.text(
        0.5, 0.01, summary_text, ha="center", va="bottom", fontsize=9, family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="gray", alpha=0.9),
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"Wrote {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
