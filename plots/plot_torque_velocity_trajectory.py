"""Plot torque vs. angular velocity for every joint as a connected, time-colored
trajectory (instead of an unordered scatter cloud), with a mass/length summary
box for each link.

Each joint's (qd, tau) samples are drawn as a single line colored by time, so
you can see the actual path traced through torque/velocity space over the
run -- e.g. one loop per gait cycle -- and how that path evolves (tightens,
drifts, grows) from the start of the run to the end.

Reads qd/tau time series from the CSV written by SimulationRunner::
writeStateTelemetry(), and mass/link-length from the MJCF model (via
plot_robot_properties.collect_links) for the annotation box.

Usage:
    python3 plots/plot_torque_velocity_trajectory.py
    python3 plots/plot_torque_velocity_trajectory.py --csv logs/debug/state_telemetry/state_telemetry.csv --show
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

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
        "--out", type=Path,
        default=Path(__file__).resolve().parent / "torque_velocity_trajectory.png",
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


def plot_phase_trajectory(ax, qd, tau, time, cmap: str = "viridis"):
    """Draw the (qd, tau) path as a single line, colored by time along its length."""
    qd = np.asarray(qd)
    tau = np.asarray(tau)
    time = np.asarray(time)

    points = np.column_stack([qd, tau]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    line_collection = LineCollection(segments, cmap=cmap, linewidth=1.0, alpha=0.85)
    line_collection.set_array(time[:-1])
    ax.add_collection(line_collection)
    ax.autoscale()
    return line_collection


def main() -> None:
    args = parse_args()
    columns = read_columns(args.csv)
    props = joint_properties(args.xml)
    time = columns["time"]

    fig, axes = plt.subplots(
        len(_LEGS), len(_JOINT_ORDER), figsize=(args.figwidth, args.figheight)
    )
    fig.suptitle("Torque vs Angular Velocity (Trajectory)")

    last_line_collection = None
    for row, leg in enumerate(_LEGS):
        for col, joint in enumerate(_JOINT_ORDER):
            ax = axes[row][col]
            qd_key = f"{leg}_{joint}_qd"
            tau_key = f"{leg}_{joint}_tau"
            if qd_key in columns and tau_key in columns:
                last_line_collection = plot_phase_trajectory(
                    ax, columns[qd_key], columns[tau_key], time
                )
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

    # Reserve the right ~6% of the figure for a dedicated colorbar axes, so it
    # can't collide with (or steal layout space from) the last subplot column.
    fig.tight_layout(rect=(0, 0.22, 0.94, 0.95))
    if last_line_collection is not None:
        cbar_ax = fig.add_axes((0.96, 0.30, 0.012, 0.55))
        fig.colorbar(last_line_collection, cax=cbar_ax, label="Time (s)")
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
