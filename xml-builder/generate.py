from __future__ import annotations

import sys
from dataclasses import dataclass, field
from typing import List, Tuple, Union

import numpy as np

import actuator as _act
from actuator import (
    ACTUATOR_CATALOG,
    ACTUATOR_BY_NAME,
    Actuator,
    ActuatorLike,
    CatalogActuator,
    GEARBOX_CATALOG,
    MOTOR_CATALOG,
)
from builder import build_xml

__all_actuators__ = (
    "Actuator", "CatalogActuator",
    "ACTUATOR_CATALOG", "MOTOR_CATALOG", "GEARBOX_CATALOG",
)

# An actuator can be referenced either by its position in ACTUATOR_CATALOG
# (int) or by its catalog "name" (str). Names are the robust choice --
# indices shift any time catalogs/actuators.json gains or loses an entry.
ActuatorRef = Union[int, str]


def _active_catalog() -> List:
    """Read the live catalog from the actuator module (may have been swapped)."""
    return _act.ACTUATOR_CATALOG


def _active_catalog_by_name() -> dict:
    return _act.ACTUATOR_BY_NAME


def _resolve(ref: ActuatorRef) -> ActuatorLike:
    if isinstance(ref, str):
        by_name = _active_catalog_by_name()
        try:
            return by_name[ref]
        except KeyError:
            raise KeyError(
                f"No actuator named {ref!r} in the active catalog "
                f"({len(_active_catalog())} entries). Available names: "
                f"{sorted(by_name)}"
            )
    cat = _active_catalog()
    idx = int(ref)
    if not (0 <= idx < len(cat)):
        raise IndexError(
            f"act_* index {idx} is out of range for the active catalog "
            f"(0..{len(cat) - 1}, {len(cat)} entries). Catalog indices shift "
            "any time catalogs/actuators.json is edited -- consider "
            "referencing actuators by NAME instead, e.g. "
            'act_abduction="MotorU12_framed+WPG:N=10.50".'
        )
    return cat[idx]


# ---------------------------------------------------------------------------
# Design specification
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DesignSpec:
    trunk_length: float
    trunk_width: float
    trunk_thickness: float
    clearance_abad_x: float
    clearance_abad_y: float
    clearance_abad_z: float
    clearance_hip_x: float
    clearance_hip_y: float
    clearance_knee_y: float
    thigh_length: float
    shank_length: float
    foot_length: float
    foot_width: float
    foot_height: float
    body_height: float   # base_Link's spawn Z, directly (no more +hip_z_drop)
    act_abduction: ActuatorRef
    act_hip: ActuatorRef
    act_knee: ActuatorRef
    act_ankle: ActuatorRef

    # --- derived ----------------------------------------------------------
    def actuators(self, catalog=None):
        """Resolve (act_abad, act_hip, act_knee, act_ankle). Each act_* field
        may be an int index or a catalog name string."""
        if catalog is not None:
            # Explicit catalog passed in -> only int indices make sense here.
            return (
                catalog[self.act_abduction],
                catalog[self.act_hip],
                catalog[self.act_knee],
                catalog[self.act_ankle],
            )
        return (
            _resolve(self.act_abduction),
            _resolve(self.act_hip),
            _resolve(self.act_knee),
            _resolve(self.act_ankle),
        )

    def per_joint_tau_max(self, catalog=None) -> np.ndarray:
        """Returns the 8-vector torque clip in joint order [R,L]x(abad,hip,knee,ankle)."""
        a, h, k, n = self.actuators(catalog)
        per_leg = np.array([a.peak_torque, h.peak_torque, k.peak_torque, n.peak_torque])
        return np.tile(per_leg, 2)

    # --- bookkeeping --------------------------------------------------------
    def vector(self) -> np.ndarray:
        """Flat float vector form used by external optimizers.

        Assumes act_* are int indices (as before) -- a catalog NAME string
        can't be cast to float, so build a from-scratch DesignSpec with int
        act_* fields if you need this (e.g. for DesignBounds.sample()).
        """
        return np.array(
            [
                self.trunk_length, self.trunk_width, self.trunk_thickness,
                self.clearance_abad_x, self.clearance_abad_y, self.clearance_abad_z,
                self.clearance_hip_x, self.clearance_hip_y,
                self.clearance_knee_y,
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
            clearance_abad_x=float(v[3]),
            clearance_abad_y=float(v[4]),
            clearance_abad_z=float(v[5]),
            clearance_hip_x=float(v[6]),
            clearance_hip_y=float(v[7]),
            clearance_knee_y=float(v[8]),
            thigh_length=float(v[9]),
            shank_length=float(v[10]),
            foot_length=float(v[11]),
            foot_width=float(v[12]),
            foot_height=float(v[13]),
            body_height=float(v[14]),
            act_abduction=int(np.clip(round(v[15]), 0, n_act - 1)),
            act_hip=     int(np.clip(round(v[16]), 0, n_act - 1)),
            act_knee=    int(np.clip(round(v[17]), 0, n_act - 1)),
            act_ankle=   int(np.clip(round(v[18]), 0, n_act - 1)),
        )

    @staticmethod
    def labels() -> List[str]:
        return [
            "trunk_L", "trunk_W", "trunk_T",
            "clr_abadX", "clr_abadY", "clr_abadZ",
            "clr_hipX", "clr_hipY",
            "clr_kneeY",
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
    clearance_abad_x: Tuple[float, float] = (0.0, 0.03)
    clearance_abad_y: Tuple[float, float] = (0.0, 0.03)
    clearance_abad_z: Tuple[float, float] = (0.02, 0.08)
    clearance_hip_x:  Tuple[float, float] = (0.005, 0.04)
    clearance_hip_y:  Tuple[float, float] = (0.005, 0.03)
    clearance_knee_y: Tuple[float, float] = (0.0, 0.03)
    thigh_length:    Tuple[float, float] = (0.22, 0.45)
    shank_length:    Tuple[float, float] = (0.22, 0.45)
    foot_length:     Tuple[float, float] = (0.14, 0.26)
    foot_width:      Tuple[float, float] = (0.05, 0.10)
    foot_height:     Tuple[float, float] = (0.015, 0.05)
    body_height:     Tuple[float, float] = (0.30, 0.85)
    # Actuator bounds default to the *current* (possibly hot-swapped) catalog.
    # Only meaningful for int-indexed act_* (see vector()/from_vector()).
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
            self.clearance_abad_x, self.clearance_abad_y, self.clearance_abad_z,
            self.clearance_hip_x, self.clearance_hip_y,
            self.clearance_knee_y,
            self.thigh_length, self.shank_length,
            self.foot_length, self.foot_width, self.foot_height,
            self.body_height,
            self.act_abduction, self.act_hip, self.act_knee, self.act_ankle,
        ]

    def integrality_mask(self) -> np.ndarray:
        """1 = integer-valued, 0 = continuous. Matches `vector()` order."""
        return np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1])

    def sample(self, rng: np.random.Generator) -> DesignSpec:
        lo_hi = self.as_pairs()
        cont = np.array(
            [rng.uniform(lo, hi) for (lo, hi) in lo_hi[:15]], dtype=float
        )
        ints = np.array(
            [rng.integers(lo, hi + 1) for (lo, hi) in lo_hi[15:]], dtype=float
        )
        return DesignSpec.from_vector(np.concatenate([cont, ints]))


# =============================================================================
# Example build (formerly the whole of generate.py)
# =============================================================================
# --- edit these --------------------------------------------------------
# This topology matches design_biped.xml/new_biped.xml (a genuine actuator
# cluster per leg: abad_act -> hip_act -> knee_act -> thigh_Link ->
# shank_Link -> ankle_act), NOT models/tron/biped.xml's own simple
# abad_Link->hip_Link->knee_Link->ankle_Link chain.
#
#   - trunk_length/width/thickness = 2x biped.xml's base_Link box <geom size>
#   - clearance_abad_z/hip_x/hip_y = the three clearance constants in the
#     abad/hip actuator-housing placement formulas (50mm/20mm/10mm, as given
#     in chat) -- everything else about abad/hip/knee housing position is
#     DERIVED from trunk dims + the picked actuators' own radii, not a free
#     mount-offset field the way biped.xml's real hip_x/hip_y/hip_z_drop work.
#   - thigh_length/shank_length = biped.xml's real hip->knee / knee->ankle
#     bone lengths -- this topology scales biped.xml's real bend DIRECTION by
#     these lengths (the bend angle itself is fixed, matching real hardware
#     CAD, not re-derived from these two numbers).
#   - foot_length/width/height = 2x biped.xml's foot <geom size>
#   - body_height = base_Link's spawn Z directly (no hip_z_drop composition
#     in this topology) -- 0.767 here matches biped.xml's real spawn height.
#
# act_abduction/act_hip/act_knee/act_ankle reference ACTUATOR_CATALOG (loaded
# from catalogs/actuators.json by default, not the catalogs/compact/
# cross-product -- see actuator.py). Referenced by NAME here rather than a
# numeric index: indices shift any time actuators.json is edited, names
# don't. (The two entries below were appended to actuators.json specifically
# to match biped.xml's real torque limits -- see that file's "notes" fields
# for what is/isn't measured data.)
if __name__ == "__main__":
    spec = DesignSpec(
        trunk_length=0.27,          # 2 * 0.135 m
        trunk_width=0.26,           # 2 * 0.13 m
        trunk_thickness=0.19,       # 2 * 0.095 m
        clearance_abad_x=0.0,       # abad x-offset clearance (on top of thickness_abad/2)
        clearance_abad_y=0.0,       # abad y-offset clearance (on top of yyy)
        clearance_abad_z=0.050,     # "clearance 1" -- abad z-offset
        clearance_hip_x=0.020,      # hip x-offset clearance
        clearance_hip_y=0.010,      # hip y-offset clearance (inside the sqrt term)
        clearance_knee_y=0.0,       # knee y-offset clearance (on top of the thigh-width gap)
        thigh_length=0.4,  # |(-0.200752, -0.286703)| in the sagittal (X-Z) plane
        shank_length=0.4,  # |(-0.017221, 0, -0.343079)|
        foot_length=0.2,            # 2 * 0.1 m
        foot_width=0.07,            # 2 * 0.035 m
        foot_height=0.03,           # 2 * 0.015 m
        body_height=1,          # base_Link spawn height, matches biped.xml exactly
        # act_abduction="MotorU12_framed+WPG:N=10.50",  # peak_torque=79.82 (biped.xml: 80)
        # act_hip="MotorU12_framed+WPG:N=10.50",        # same -- biped.xml uses one torque class for ab/ad + hip + knee
        # act_knee="MotorU12_framed+WPG:N=10.50",       # same
        # act_ankle="MotorMN8014_framed+CPG:N=4.00",    # peak_torque=19.97 (biped.xml: 20)
        act_abduction=3,   # TMotor-AK80-9, 24.00 Nm (was "MotorU12_framed+WPG:N=10.50")
        act_hip=3,         # TMotor-AK80-9, 24.00 Nm — much lighter/weaker than the original 79.82 Nm
        act_knee=3,        # TMotor-AK80-9, 24.00 Nm
        act_ankle=4,       # TMotor-AK80-6, 12.00 Nm (was "MotorMN8014_framed+CPG:N=4.00", not in this catalog)
    )
    
    # -------------------------------------------------------------------------

    try:
        spec.actuators()   # resolves all four act_* refs (name or index) up front
    except (KeyError, IndexError) as e:
        raise SystemExit(f"actuator resolution failed: {e}")

    out_path = sys.argv[1] if len(sys.argv) > 1 else "design.xml"
    xml = build_xml(spec)
    with open(out_path, "w") as f:
        f.write(xml)

    a_act, h_act, k_act, n_act = spec.actuators()
    print(f"Wrote {out_path}")
    print(f"Ab/ad actuator: {a_act.name} (peak_torque={a_act.peak_torque:.2f} Nm)")
    print(f"Hip actuator:   {h_act.name} (peak_torque={h_act.peak_torque:.2f} Nm)")
    print(f"Knee actuator:  {k_act.name} (peak_torque={k_act.peak_torque:.2f} Nm)")
    print(f"Ankle actuator: {n_act.name} (peak_torque={n_act.peak_torque:.2f} Nm)")
    print(f"View it with: python view_joints.py {out_path}")