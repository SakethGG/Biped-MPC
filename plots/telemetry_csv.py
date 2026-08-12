"""Shared CSV loader for state_telemetry.csv (written by SimulationRunner::writeStateTelemetry)."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List


def read_columns(csv_path: Path) -> Dict[str, List[float]]:
    """Read a telemetry CSV into a dict mapping column name -> list of floats."""
    if not csv_path.exists():
        raise FileNotFoundError(
            f"Telemetry CSV not found: {csv_path}\n"
            "Generate it by running the sim with, e.g.:\n"
            "  CONVEXMPC_STATE_TELEMETRY=1 ./build/apps/main t n\n"
            "(set CONVEXMPC_STATE_TELEMETRY to a custom path instead of 1 to control where it's written,\n"
            "and CONVEXMPC_STATE_TELEMETRY_DT to control the sample interval in seconds)."
        )

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        columns: Dict[str, List[float]] = {name: [] for name in (reader.fieldnames or [])}
        for row in reader:
            for name, value in row.items():
                columns[name].append(float(value))

    return columns


def default_state_telemetry_path() -> Path:
    return Path(__file__).resolve().parents[1] / "logs" / "debug" / "state_telemetry" / "state_telemetry.csv"
