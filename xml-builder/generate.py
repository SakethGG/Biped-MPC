from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
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
    """Design variables for one biped build.

    Fields are grouped in two tiers, matching ``doc.py``'s naming convention
    (``clearance_{source}2{target}_{axis}``, read as "clearance from source
    to target, along axis"):

    - Free fields (no Python default): swept by ``DesignBounds``/``vector()``/
      ``from_vector()`` -- the actual design-search space.
    - Fixed-constant fields (Python default below, "##constant" in the source
      notes): hardware constants formalized as named fields so the placement
      formulas can reference them, but NOT part of the optimizer's vector --
      omitting one at construction falls back to the default below.
    """

    # --- Trunk ----------------------------------------------------------
    trunk_length: float
    trunk_width: float
    trunk_thickness: float

    # --- ABAD actuator (base_Link -> abad_{SIDE}_act) --------------------
    clearance_trunk2abadAct_z: float     # abad z-offset clearance

    # --- Hip actuator (abad_{SIDE}_act -> hip_{SIDE}_act) -----------------
    clearance_abadAct2hipAct_x: float    # hip x-offset clearance
    clearance_abadAct2hipAct_y: float    # hip y-offset clearance (inside the sqrt term)

    # --- Knee actuator (hip_{SIDE}_act -> knee_{SIDE}_act) -----------------
    clearance_hipAct2kneeAct_y: float    # knee y-offset clearance (on top of the thigh-width gap)

    # --- Thigh link (knee_{SIDE}_act -> thigh_{SIDE}_Link) ------------------
    thigh_length: float

    # --- Shank link ---------------------------------------------------------
    shank_length: float

    # --- Foot link (ankle_{SIDE}_act -> foot_{SIDE}_Link) --------------------
    # foot_width/foot_height are NOT DesignSpec fields -- the foot box's
    # width/height cross-section is a fixed builder.py constant,
    # FOOT_BOX_HALF, same treatment as THIGH_BOX_HALF/SHANK_BOX_HALF.
    foot_length: float

    body_height: float   # base_Link's spawn Z, directly (no more +hip_z_drop)

    # --- Actuator picks (catalog index or name) ------------------------------
    act_abduction: ActuatorRef
    act_hip: ActuatorRef
    act_knee: ActuatorRef
    act_ankle: ActuatorRef

    # --- Fixed-constant fields (see class docstring) -- excluded from
    # DesignBounds/vector()/from_vector()/labels()/sample(). ------------------
    clearance_trunk2abadAct_x: float = 0.0
    clearance_trunk2abadAct_y: float = 0.0
    clearance_abadAct2hipAct_z: float = 0.0
    clearance_hipAct2kneeAct_x: float = 0.0
    clearance_hipAct2kneeAct_z: float = 0.0
    # thigh_width/thigh_thickness/shank_width/shank_thickness removed --
    # never wired into any placement/sizing formula, fully superseded by
    # builder.py's THIGH_BOX_HALF/SHANK_BOX_HALF (constant_params.json).
    clearance_kneeAct2thigh_x: float = 0.0
    clearance_kneeAct2thigh_y: float = 0.0
    clearance_kneeAct2thigh_z: float = 0.0
    # NOTE: doc.py's draft called this actuator "footAct" -- it's actually
    # the ANKLE actuator (act_ankle); named clearance_shank2ankleAct_* /
    # clearance_ankleAct2foot_* here to avoid clashing with the foot LINK.
    clearance_shank2ankleAct_x: float = 0.0
    clearance_shank2ankleAct_y: float = 0.0
    clearance_shank2ankleAct_z: float = 0.0
    clearance_ankleAct2foot_x: float = 0.0
    clearance_ankleAct2foot_y: float = 0.0
    clearance_ankleAct2foot_z: float = 0.03   # must stay positive -- foot sits below the ankle joint

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
        """Flat float vector form used by external optimizers. Only the FREE
        fields (see class docstring) are included -- fixed-constant fields
        aren't part of the design-search space and fall back to their Python
        default when `from_vector` reconstructs a DesignSpec.

        Assumes act_* are int indices (as before) -- a catalog NAME string
        can't be cast to float, so build a from-scratch DesignSpec with int
        act_* fields if you need this (e.g. for DesignBounds.sample()).
        """
        return np.array(
            [
                self.trunk_length, self.trunk_width, self.trunk_thickness,
                self.clearance_trunk2abadAct_z,
                self.clearance_abadAct2hipAct_x, self.clearance_abadAct2hipAct_y,
                self.clearance_hipAct2kneeAct_y,
                self.thigh_length, self.shank_length,
                self.foot_length,
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
            clearance_trunk2abadAct_z=float(v[3]),
            clearance_abadAct2hipAct_x=float(v[4]),
            clearance_abadAct2hipAct_y=float(v[5]),
            clearance_hipAct2kneeAct_y=float(v[6]),
            thigh_length=float(v[7]),
            shank_length=float(v[8]),
            foot_length=float(v[9]),
            body_height=float(v[10]),
            act_abduction=int(np.clip(round(v[11]), 0, n_act - 1)),
            act_hip=     int(np.clip(round(v[12]), 0, n_act - 1)),
            act_knee=    int(np.clip(round(v[13]), 0, n_act - 1)),
            act_ankle=   int(np.clip(round(v[14]), 0, n_act - 1)),
        )

    @staticmethod
    def labels() -> List[str]:
        return [
            "trunk_L", "trunk_W", "trunk_T",
            "clr_t2aZ",
            "clr_a2hX", "clr_a2hY",
            "clr_h2kY",
            "thigh_L", "shank_L",
            "foot_L",
            "body_h",
            "act_abd", "act_hip", "act_kne", "act_ank",
        ]


# ---------------------------------------------------------------------------
# Search bounds
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class DesignBounds:
    """Inclusive [lo, hi] on every FREE design variable (see DesignSpec's
    class docstring for the free/fixed-constant split -- fixed-constant
    fields have no entry here since they aren't swept)."""

    trunk_length:    Tuple[float, float] = (0.20, 0.40)
    trunk_width:     Tuple[float, float] = (0.18, 0.34)
    trunk_thickness: Tuple[float, float] = (0.12, 0.26)
    clearance_trunk2abadAct_z:  Tuple[float, float] = (0.02, 0.08)
    clearance_abadAct2hipAct_x: Tuple[float, float] = (0.005, 0.04)
    clearance_abadAct2hipAct_y: Tuple[float, float] = (0.005, 0.03)
    clearance_hipAct2kneeAct_y: Tuple[float, float] = (0.0, 0.03)
    thigh_length:    Tuple[float, float] = (0.18, 0.45)
    shank_length:    Tuple[float, float] = (0.22, 0.45)
    foot_length:     Tuple[float, float] = (0.14, 0.26)
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
            self.clearance_trunk2abadAct_z,
            self.clearance_abadAct2hipAct_x, self.clearance_abadAct2hipAct_y,
            self.clearance_hipAct2kneeAct_y,
            self.thigh_length, self.shank_length,
            self.foot_length,
            self.body_height,
            self.act_abduction, self.act_hip, self.act_knee, self.act_ankle,
        ]

    def integrality_mask(self) -> np.ndarray:
        """1 = integer-valued, 0 = continuous. Matches `vector()` order."""
        return np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1])

    def sample(self, rng: np.random.Generator) -> DesignSpec:
        lo_hi = self.as_pairs()
        cont = np.array(
            [rng.uniform(lo, hi) for (lo, hi) in lo_hi[:11]], dtype=float
        )
        ints = np.array(
            [rng.integers(lo, hi + 1) for (lo, hi) in lo_hi[11:]], dtype=float
        )
        return DesignSpec.from_vector(np.concatenate([cont, ints]))


# =============================================================================
# Example build -- loads every DesignSpec field from variable_params.json
# =============================================================================
# This topology matches design_biped.xml/new_biped.xml (a genuine actuator
# cluster per leg: abad_act -> hip_act -> knee_act -> thigh_Link ->
# shank_Link -> ankle_act), NOT models/tron/biped.xml's own simple
# abad_Link->hip_Link->knee_Link->ankle_Link chain.
#
# Every DesignSpec field -- both the FREE fields (trunk dims, the four free
# clearances, thigh/shank/foot lengths, body_height, act_*) and the
# fixed-constant ones (see DesignSpec's class docstring above) -- lives in
# variable_params.json as one flat JSON object, edited THERE rather than in
# this file. (builder.py's own module-level constants -- box cross-sections,
# bend angles, densities, etc. -- live in the separate constant_params.json,
# see builder.py's module docstring.)
#
# act_abduction/act_hip/act_knee/act_ankle in variable_params.json are raw
# catalog INDICES into ACTUATOR_CATALOG (loaded from catalogs/actuators.json
# by default, not the catalogs/compact/ cross-product -- see actuator.py) --
# an index shifts meaning any time actuators.json gains or loses an entry, so
# treat that file as append-only if you're relying on a specific index.
VARIABLE_PARAMS_PATH = Path(__file__).parent / "variable_params.json"


def load_design_spec(path: Path = VARIABLE_PARAMS_PATH) -> DesignSpec:
    """Build a DesignSpec from a variable_params.json-shaped file -- every
    DesignSpec field as one flat JSON object (see VARIABLE_PARAMS_PATH)."""
    with open(path) as f:
        data = json.load(f)
    return DesignSpec(**data)


if __name__ == "__main__":
    spec = load_design_spec()

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