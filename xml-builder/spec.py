"""Decision variables and catalog plumbing for the biped design space.

Design variables
----------------
Continuous (per robot, mirrored across the 2 legs):
    trunk_length, trunk_width, trunk_thickness  -- base/pelvis box outer dims [m]
    hip_x, hip_y, hip_z_drop                     -- ab/ad joint offset from base center [m]
    thigh_length, shank_length                   -- leg link lengths [m]
    foot_length, foot_width, foot_height          -- foot box outer dims [m]
    body_height                                   -- standing hip-to-ankle vertical drop [m]

Discrete (per joint type, mirrored across the 2 legs):
    act_abduction, act_hip, act_knee, act_ankle  -- index into ACTUATOR_CATALOG

This is a direct port of the quadruped_xml_builder project's ``spec.py`` to a
2-legged, 4-joints-per-leg (ab/ad, hip, knee, ankle) topology matching
``models/tron/biped.xml``. The actuator catalog machinery (``actuator.py`` +
``catalogs/``) is reused unchanged -- it has no notion of how many legs a
robot has.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

import actuator as _act
from actuator import (
    ACTUATOR_CATALOG,
    Actuator,
    CatalogActuator,
    GEARBOX_CATALOG,
    MOTOR_CATALOG,
)

__all_actuators__ = (
    "Actuator", "CatalogActuator",
    "ACTUATOR_CATALOG", "MOTOR_CATALOG", "GEARBOX_CATALOG",
)


def _active_catalog() -> List:
    """Read the live catalog from the actuator module (may have been swapped)."""
    return _act.ACTUATOR_CATALOG


# ---------------------------------------------------------------------------
# Design specification
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DesignSpec:
    trunk_length: float
    trunk_width: float
    trunk_thickness: float
    hip_x: float
    hip_y: float
    hip_z_drop: float
    thigh_length: float
    shank_length: float
    foot_length: float
    foot_width: float
    foot_height: float
    body_height: float
    act_abduction: int
    act_hip: int
    act_knee: int
    act_ankle: int

    # --- derived ----------------------------------------------------------
    def actuators(self, catalog=None):
        cat = catalog if catalog is not None else _active_catalog()
        return (
            cat[self.act_abduction],
            cat[self.act_hip],
            cat[self.act_knee],
            cat[self.act_ankle],
        )

    def per_joint_tau_max(self, catalog=None) -> np.ndarray:
        """Returns the 8-vector torque clip in joint order [R,L]x(abad,hip,knee,ankle)."""
        a, h, k, n = self.actuators(catalog)
        per_leg = np.array([a.peak_torque, h.peak_torque, k.peak_torque, n.peak_torque])
        return np.tile(per_leg, 2)

    # --- bookkeeping ------------------------------------------------------
    def vector(self) -> np.ndarray:
        """Flat float vector form used by external optimizers (integers cast to float)."""
        return np.array(
            [
                self.trunk_length, self.trunk_width, self.trunk_thickness,
                self.hip_x, self.hip_y, self.hip_z_drop,
                self.thigh_length, self.shank_length,
                self.foot_length, self.foot_width, self.foot_height,
                self.body_height,
                self.act_abduction, self.act_hip, self.act_knee, self.act_ankle,
            ],
            dtype=float,
        )

    @staticmethod
    def from_vector(v: np.ndarray) -> "DesignSpec":
        v = np.asarray(v)
        n_act = len(_active_catalog())
        return DesignSpec(
            trunk_length=float(v[0]),
            trunk_width=float(v[1]),
            trunk_thickness=float(v[2]),
            hip_x=float(v[3]),
            hip_y=float(v[4]),
            hip_z_drop=float(v[5]),
            thigh_length=float(v[6]),
            shank_length=float(v[7]),
            foot_length=float(v[8]),
            foot_width=float(v[9]),
            foot_height=float(v[10]),
            body_height=float(v[11]),
            act_abduction=int(np.clip(round(v[12]), 0, n_act - 1)),
            act_hip=     int(np.clip(round(v[13]), 0, n_act - 1)),
            act_knee=    int(np.clip(round(v[14]), 0, n_act - 1)),
            act_ankle=   int(np.clip(round(v[15]), 0, n_act - 1)),
        )

    @staticmethod
    def labels() -> List[str]:
        return [
            "trunk_L", "trunk_W", "trunk_T",
            "hip_x", "hip_y", "hip_z_drop",
            "thigh_L", "shank_L",
            "foot_L", "foot_W", "foot_H",
            "body_h",
            "act_abd", "act_hip", "act_kne", "act_ank",
        ]


# ---------------------------------------------------------------------------
# Search bounds
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DesignBounds:
    """Inclusive [lo, hi] on every design variable."""

    trunk_length:    Tuple[float, float] = (0.20, 0.40)
    trunk_width:     Tuple[float, float] = (0.18, 0.34)
    trunk_thickness: Tuple[float, float] = (0.12, 0.26)
    hip_x:           Tuple[float, float] = (0.00, 0.08)
    hip_y:           Tuple[float, float] = (0.05, 0.14)
    hip_z_drop:      Tuple[float, float] = (0.10, 0.28)
    thigh_length:    Tuple[float, float] = (0.22, 0.45)
    shank_length:    Tuple[float, float] = (0.22, 0.45)
    foot_length:     Tuple[float, float] = (0.14, 0.26)
    foot_width:      Tuple[float, float] = (0.05, 0.10)
    foot_height:     Tuple[float, float] = (0.015, 0.05)
    body_height:     Tuple[float, float] = (0.30, 0.85)
    # Actuator bounds default to the *current* (possibly hot-swapped) catalog.
    act_abduction: Tuple[int, int] = field(
        default_factory=lambda: (0, len(_active_catalog()) - 1))
    act_hip:       Tuple[int, int] = field(
        default_factory=lambda: (0, len(_active_catalog()) - 1))
    act_knee:      Tuple[int, int] = field(
        default_factory=lambda: (0, len(_active_catalog()) - 1))
    act_ankle:     Tuple[int, int] = field(
        default_factory=lambda: (0, len(_active_catalog()) - 1))

    def as_pairs(self) -> List[Tuple[float, float]]:
        return [
            self.trunk_length, self.trunk_width, self.trunk_thickness,
            self.hip_x, self.hip_y, self.hip_z_drop,
            self.thigh_length, self.shank_length,
            self.foot_length, self.foot_width, self.foot_height,
            self.body_height,
            self.act_abduction, self.act_hip, self.act_knee, self.act_ankle,
        ]

    def integrality_mask(self) -> np.ndarray:
        """1 = integer-valued, 0 = continuous. Matches `vector()` order."""
        return np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1])

    def sample(self, rng: np.random.Generator) -> DesignSpec:
        lo_hi = self.as_pairs()
        cont = np.array(
            [rng.uniform(lo, hi) for (lo, hi) in lo_hi[:12]], dtype=float
        )
        ints = np.array(
            [rng.integers(lo, hi + 1) for (lo, hi) in lo_hi[12:]], dtype=float
        )
        return DesignSpec.from_vector(np.concatenate([cont, ints]))
