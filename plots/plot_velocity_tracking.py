"""Plot commanded ("Desired") vs. actual CoM velocity vs. time, x and y.

Reads cmd_x_dot/cmd_y_dot (the raw x_dot/y_dot sent to the controller, before
its ~0.5s low-pass filter) and com_vx/com_vy (actual measured CoM velocity)
from the CSV written by SimulationRunner::writeStateTelemetry().

Usage:
    python3 plots/plot_velocity_tracking.py
    python3 plots/plot_velocity_tracking.py --csv logs/debug/state_telemetry/state_telemetry.csv --show
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_csv import default_state_telemetry_path, read_columns  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=default_state_telemetry_path())
    parser.add_argument(
        "--out", type=Path, default=Path(__file__).resolve().parent / "velocity_tracking.png"
    )
    parser.add_argument("--show", action="store_true", help="Also display the plot interactively")
    parser.add_argument("--figwidth", type=float, default=7.0)
    parser.add_argument("--figheight", type=float, default=7.0)
    return parser.parse_args()


def plot_axis(ax, time, actual, desired, label: str) -> None:
    ax.plot(time, actual, color="tab:blue", linewidth=1.2, label="Actual Velocity")
    ax.plot(time, desired, color="red", linestyle=":", linewidth=1.5, label="Desired Velocity")
    ax.set_ylabel(f"CoM velocity {label} (m/s)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", framealpha=1.0)


def main() -> None:
    args = parse_args()
    columns = read_columns(args.csv)
    time = columns["time"]

    fig, axes = plt.subplots(2, 1, figsize=(args.figwidth, args.figheight))
    fig.suptitle("Velocity Tracking")

    plot_axis(axes[0], time, columns["com_vx"], columns["cmd_x_dot"], "X")
    plot_axis(axes[1], time, columns["com_vy"], columns["cmd_y_dot"], "Y")

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print(f"Wrote {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
