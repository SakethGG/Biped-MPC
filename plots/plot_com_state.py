"""Plot COM (reduced-body) position and velocity in the fixed world frame vs. time,
plus the COM's 3D path through space.

Reads the CSV written by SimulationRunner::writeStateTelemetry(). Generate it by running:

    CONVEXMPC_STATE_TELEMETRY=1 ./build/apps/main t n
    # or CONVEXMPC_STATE_TELEMETRY=some/path.csv to control where it's written
    # and CONVEXMPC_STATE_TELEMETRY_DT=0.01 to control the sample interval (seconds)

Usage:
    python3 plots/plot_com_state.py
    python3 plots/plot_com_state.py --csv logs/debug/state_telemetry/state_telemetry.csv --show
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parent))
from telemetry_csv import default_state_telemetry_path, read_columns

_AXES = ["x", "y", "z"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=default_state_telemetry_path())
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "com_state.png",
        help="Output image path for the position/velocity time-series figure",
    )
    parser.add_argument(
        "--out-3d",
        type=Path,
        default=Path(__file__).resolve().parent / "com_trajectory_3d.png",
        help="Output image path for the 3D COM trajectory figure",
    )
    parser.add_argument("--show", action="store_true", help="Also display the plots interactively")
    parser.add_argument("--figwidth", type=float, default=11.0)
    parser.add_argument("--figheight", type=float, default=6.5)
    return parser.parse_args()


def plot_time_series(columns, out_path: Path, figwidth: float, figheight: float) -> None:
    time = columns["time"]
    fig, axes = plt.subplots(2, 3, figsize=(figwidth, figheight), sharex=True)
    fig.suptitle("COM position and velocity vs. time (world frame)")

    for col_idx, axis in enumerate(_AXES):
        pos_ax = axes[0][col_idx]
        pos_ax.plot(time, columns[f"com_{axis}"], color="tab:blue")
        pos_ax.set_title(f"{axis.upper()}")
        pos_ax.set_ylabel("position [m]" if col_idx == 0 else "")
        pos_ax.grid(True, alpha=0.3)

        vel_ax = axes[1][col_idx]
        vel_ax.plot(time, columns[f"com_v{axis}"], color="tab:orange")
        vel_ax.set_ylabel("velocity [m/s]" if col_idx == 0 else "")
        vel_ax.set_xlabel("time [s]")
        vel_ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"Wrote {out_path}")


def plot_3d_trajectory(columns, out_path: Path) -> None:
    fig = plt.figure(figsize=(7.0, 6.0))
    ax = fig.add_subplot(projection="3d")
    ax.plot(columns["com_x"], columns["com_y"], columns["com_z"], color="tab:blue")
    ax.scatter(columns["com_x"][0], columns["com_y"][0], columns["com_z"][0], color="green", label="start")
    ax.scatter(columns["com_x"][-1], columns["com_y"][-1], columns["com_z"][-1], color="red", label="end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("COM trajectory (world frame)")
    ax.legend()

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"Wrote {out_path}")


def main() -> None:
    args = parse_args()
    columns = read_columns(args.csv)

    plot_time_series(columns, args.out, args.figwidth, args.figheight)
    plot_3d_trajectory(columns, args.out_3d)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
