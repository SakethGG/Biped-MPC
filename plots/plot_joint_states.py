"""Plot per-joint position, velocity, torque, and acceleration vs. time.

Reads the CSV written by SimulationRunner::writeStateTelemetry() (columns named
"<left|right>_<joint>_<q|qd|tau|qacc>"). Generate that CSV by running the sim with:

    CONVEXMPC_STATE_TELEMETRY=1 ./build/apps/main t n
    # or CONVEXMPC_STATE_TELEMETRY=some/path.csv to control where it's written
    # and CONVEXMPC_STATE_TELEMETRY_DT=0.01 to control the sample interval (seconds)

Usage:
    python3 plots/plot_joint_states.py
    python3 plots/plot_joint_states.py --csv logs/debug/state_telemetry/state_telemetry.csv --show
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_csv import default_state_telemetry_path, read_columns

_QUANTITIES = [
    ("q", "Position [rad]"),
    ("qd", "Velocity [rad/s]"),
    ("tau", "Torque [N*m]"),
    ("qacc", "Acceleration [rad/s^2]"),
]
_LEGS = ["left", "right"]
_COLUMN_PATTERN = re.compile(r"^(left|right)_(.+)_(q|qd|tau|qacc)$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=default_state_telemetry_path())
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "joint_states.png",
        help="Output image path",
    )
    parser.add_argument("--show", action="store_true", help="Also display the plot interactively")
    parser.add_argument("--figwidth", type=float, default=13.0)
    parser.add_argument("--figheight", type=float, default=11.0)
    return parser.parse_args()


def joints_per_leg(columns) -> dict:
    """Discover joint names per leg, preserving the order they appear in the CSV header."""
    joints = {leg: [] for leg in _LEGS}
    for name in columns:
        match = _COLUMN_PATTERN.match(name)
        if match is None:
            continue
        leg, joint, _quantity = match.groups()
        if joint not in joints[leg]:
            joints[leg].append(joint)
    return joints


def main() -> None:
    args = parse_args()
    columns = read_columns(args.csv)
    time = columns["time"]
    joints = joints_per_leg(columns)

    fig, axes = plt.subplots(
        len(_QUANTITIES), len(_LEGS), figsize=(args.figwidth, args.figheight), sharex=True
    )
    fig.suptitle("Joint state vs. time")

    for col_idx, leg in enumerate(_LEGS):
        for row_idx, (quantity, ylabel) in enumerate(_QUANTITIES):
            ax = axes[row_idx][col_idx]
            for joint in joints[leg]:
                column_name = f"{leg}_{joint}_{quantity}"
                if column_name in columns:
                    ax.plot(time, columns[column_name], label=joint)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
            if row_idx == 0:
                ax.set_title(f"{leg.capitalize()} leg")
                ax.legend(loc="upper right", fontsize=8)
            if row_idx == len(_QUANTITIES) - 1:
                ax.set_xlabel("time [s]")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"Wrote {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
