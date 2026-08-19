"""Motor + planetary-gearbox actuator model from the official COMPAct repo.

Reference
---------
A. Singh, D. Kapa, S. Joshi, S. Kolathaya.  *"COMPAct: Computational
Optimization and Automated Modular Design of Planetary Actuators"*, ICRA 2026.
https://arxiv.org/abs/2510.07197

Official code (MIT licensed):
    https://github.com/singhaman1750/COMPAct

We bundle the published artefacts from that repo under ``catalogs/compact/``:

* ``compact_config.json`` — motor table (7 BLDC motors with Kv, current,
  voltage, mass, diameter, …) and material/cost gains.
* ``{sspg,cpg,wpg,dspg}_params.json`` — per-topology optimisation
  bounds (``GEAR_RATIO_MIN``, ``GEAR_RATIO_MAX``, ``GEAR_RATIO_STEP``,
  tooth-count and module ranges).
* ``{SSPG,CPG,WPG,DSPG}_MAD.csv`` — the published *mass-and-diameter*
  Pareto rows: each row is a tooth-count-feasible gearbox at a specific
  ratio, with the optimised gearbox mass, theoretical efficiency, peak
  torque, cost, actuator width.

The actuator catalog is built as the cross-product

    Actuator  =  Motor  ⊗  GearboxType  ⊗  ratio

where the **ratios are taken from the MAD CSVs**, i.e. every entry
corresponds to a real, physically realisable design that the COMPAct
optimiser produced. Joint-side properties (peak torque, peak speed, mass,
cost) are then derived for the *selected* motor by scaling the published
U8-based rows. This avoids the previous parametric model which generated
ratios that did not correspond to feasible tooth counts (e.g. CPG N=22).
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple, Union


# ---------------------------------------------------------------------------
# Bundled JSON catalog locations (can be overridden via set_active_catalog).
# ---------------------------------------------------------------------------
_CATALOGS_DIR  = Path(__file__).parent / "catalogs"
_COMPACT_DIR   = _CATALOGS_DIR / "compact"
DEFAULT_MOTORS_JSON      = _CATALOGS_DIR / "motors.json"
DEFAULT_ACTUATORS_JSON   = _CATALOGS_DIR / "actuators.json"
DEFAULT_COMPACT_CONFIG   = _COMPACT_DIR / "compact_config.json"
DEFAULT_MAD_CSV_TEMPLATE = str(_COMPACT_DIR / "{NAME}_MAD.csv")  # NAME in {SSPG,CPG,WPG,DSPG}
DEFAULT_GB_PARAMS_TEMPLATE = str(_COMPACT_DIR / "{name}_params.json")  # name in {sspg,cpg,wpg,dspg}


# ---------------------------------------------------------------------------
# Motor catalog
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Motor:
    """A simple flat / pancake BLDC motor model (joint-side properties below)."""

    name: str
    Kt: float            # [N·m/A]   torque constant
    R: float             # [Ω]       phase resistance (estimated when unavailable)
    Imax: float          # [A]       peak (continuous) current
    omega_nl: float      # [rad/s]   no-load motor-side speed at rated voltage
    rotor_inertia: float # [kg·m²]
    mass: float          # [kg]
    cost: float          # [$]
    diameter: Optional[float] = None  # [m] motor body outer diameter, when known
    length: Optional[float] = None    # [m] motor body axial length, when known

    @property
    def tau_peak_motor(self) -> float:
        return self.Kt * self.Imax


def _kv_to_kt(kv_rpm_per_volt: float) -> float:
    """Standard SI conversion: Kt [Nm/A] = 60 / (2π · Kv [rpm/V])."""
    return 60.0 / (2.0 * math.pi * kv_rpm_per_volt)


def _estimate_R(rated_voltage: float, max_current: float,
                motor_eff: float = 0.85) -> float:
    """Heuristic phase resistance when not specified.

    Assumes ~``motor_eff`` electrical-to-mechanical efficiency at the rated
    operating point, so the IR drop is ``V·(1-η)`` at the rated current.
    """
    return rated_voltage * (1.0 - motor_eff) / max(max_current, 1e-3)


def _cost_from_mass(mass_kg: float, dollars_per_kg: float = 380.0) -> float:
    """Rough $-per-kg surrogate for catalog motors (no official price list)."""
    return float(mass_kg * dollars_per_kg)


def load_motor_catalog_json(path: Union[str, Path]) -> List[Motor]:
    """Load BLDC motor catalog from a JSON file (see catalogs/motors.json).

    Accepts either our own schema (``"motors": [...]`` with explicit ``Kt`` /
    ``R`` / ``omega_nl`` fields) **or** the official COMPAct schema
    (``Motors: {name: {Kv, ratedVoltage, maxContinuousCurrent, mass, ...}}``).
    """
    data = json.loads(Path(path).read_text())

    # ---- our schema: {"motors": [...]} ---------------------------------
    if isinstance(data, dict) and "motors" in data and isinstance(data["motors"], list):
        items = data["motors"]
        out: List[Motor] = []
        for d in items:
            out.append(Motor(
                name=str(d["name"]),
                Kt=float(d["Kt"]), R=float(d["R"]),
                Imax=float(d["Imax"]), omega_nl=float(d["omega_nl"]),
                rotor_inertia=float(d.get("rotor_inertia", 0.0)),
                mass=float(d["mass"]), cost=float(d["cost"]),
                diameter=(float(d["diameter"]) if "diameter" in d else None),
                length=(float(d["length"]) if "length" in d else None),
            ))
        return out

    # ---- official COMPAct schema: top-level "Motors" dict ---------------
    if isinstance(data, dict) and "Motors" in data and isinstance(data["Motors"], dict):
        out = []
        for name, d in data["Motors"].items():
            kv = float(d["Kv"])
            v_rated = float(d["ratedVoltage"])
            i_max = float(d["maxContinuousCurrent"])
            kt = _kv_to_kt(kv)
            omega_nl = kv * v_rated * (2.0 * math.pi / 60.0)
            r = _estimate_R(v_rated, i_max)
            # "dia"/"length" (mm) are the motor body's own outer diameter/axial
            # length in the published config -- convert to metres to match our
            # diameter/length convention.
            out.append(Motor(
                name=name, Kt=kt, R=r, Imax=i_max, omega_nl=omega_nl,
                rotor_inertia=0.0,
                mass=float(d["mass"]),
                cost=_cost_from_mass(float(d["mass"])),
                diameter=(float(d["dia"]) / 1000.0 if "dia" in d else None),
                length=(float(d["length"]) / 1000.0 if "length" in d else None),
            ))
        return out

    if isinstance(data, list):
        out = []
        for d in data:
            out.append(Motor(
                name=str(d["name"]),
                Kt=float(d["Kt"]), R=float(d["R"]),
                Imax=float(d["Imax"]), omega_nl=float(d["omega_nl"]),
                rotor_inertia=float(d.get("rotor_inertia", 0.0)),
                mass=float(d["mass"]), cost=float(d["cost"]),
                diameter=(float(d["diameter"]) if "diameter" in d else None),
                length=(float(d["length"]) if "length" in d else None),
            ))
        return out

    raise ValueError(f"Unrecognised motor JSON schema in {path}")


# Built-in fallback used only if neither JSON loads.
_BUILTIN_MOTORS: List[Motor] = [
    Motor("M-mini",  Kt=0.060, R=0.18, Imax=18.0, omega_nl=310.0,
          rotor_inertia=3.2e-5, mass=0.22, cost=180.0),
    Motor("M-mid",   Kt=0.085, R=0.11, Imax=25.0, omega_nl=270.0,
          rotor_inertia=7.5e-5, mass=0.32, cost=240.0),
    Motor("M-big",   Kt=0.110, R=0.083, Imax=33.0, omega_nl=240.0,
          rotor_inertia=1.4e-4, mass=0.48, cost=360.0),
]


def _load_default_motor_catalog() -> List[Motor]:
    """Prefer the official COMPAct config; fall back to our JSON; then builtin."""
    for path in (DEFAULT_COMPACT_CONFIG, DEFAULT_MOTORS_JSON):
        try:
            return load_motor_catalog_json(path)
        except Exception:
            continue
    return list(_BUILTIN_MOTORS)


MOTOR_CATALOG: List[Motor] = _load_default_motor_catalog()


# ---------------------------------------------------------------------------
# Gearbox topologies (parameter bounds loaded from official config_files/*.json)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MADRow:
    """One row of a published COMPAct MAD (mass-and-diameter) CSV.

    The row is the result of COMPAct's per-ratio tooth-count optimisation for
    a single motor (MotorU8 in the published runs). It captures everything
    needed to characterise a *physically realisable* gearbox at the given
    integer-tooth ratio: optimised gearbox mass, theoretical efficiency, peak
    output torque, actuator width.
    """
    ratio: float
    mass_kg: float           # gearbox-only mass (without motor)
    eff: float               # theoretical efficiency from gear-pair model
    peak_torque_Nm: float    # joint-side, for the reference motor U8
    width_m: float           # actuator outer width
    cost_unit: float         # COMPAct dimensionless objective value
    num_planet: int = 0      # planet count (informational)


def load_mad_csv(path: Union[str, Path]) -> List[MADRow]:
    """Parse one of the bundled ``{SSPG,CPG,WPG,DSPG}_MAD.csv`` files."""
    rows: List[MADRow] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append(MADRow(
                    ratio=float(r["gearRatio"]),
                    mass_kg=float(r["mass"]),
                    eff=float(r["eff"]),
                    peak_torque_Nm=float(r["peakTorque"]),
                    width_m=float(r["Actuator_width"]) / 1000.0,
                    cost_unit=float(r.get("Cost", 0.0) or 0.0),
                    num_planet=int(float(r.get("numPlanet",
                                               r.get("numPlanet1", 0)) or 0)),
                ))
            except (KeyError, ValueError):
                continue
    rows.sort(key=lambda x: x.ratio)
    return rows


@dataclass(frozen=True)
class GearboxType:
    """A planetary gearbox topology backed by official COMPAct artefacts.

    ``ratio_min`` / ``ratio_max`` / ``ratio_step`` come from the topology's
    ``*_params.json``. ``mad_rows`` is the discrete table of feasible designs
    produced by COMPAct's published MAD optimisation — these are the *only*
    physically realisable ratios we draw from.
    """
    name: str                   # SSPG / CPG / WPG / DSPG
    ratio_min: float
    ratio_max: float
    ratio_step: float
    mad_rows: Tuple[MADRow, ...]
    ref_motor_tau_peak: float   # τ_peak of the motor used in the published MAD run

    @property
    def ratio_min_published(self) -> float:
        return self.mad_rows[0].ratio if self.mad_rows else self.ratio_min

    @property
    def ratio_max_published(self) -> float:
        return self.mad_rows[-1].ratio if self.mad_rows else self.ratio_max

    def closest_row(self, ratio: float) -> MADRow:
        return min(self.mad_rows, key=lambda r: abs(r.ratio - ratio))


# Default reference motor used by the COMPAct MAD CSVs.  The repo's published
# runs scale gearbox sizing and peakTorque around MotorU8_framed.  We compute
# the joint-side peak torque of *other* motors by scaling.
_REF_MOTOR_NAME_FOR_MAD = "MotorU8_framed"


def _ref_motor_tau_peak(motors: List[Motor]) -> float:
    for m in motors:
        if m.name == _REF_MOTOR_NAME_FOR_MAD:
            return m.tau_peak_motor
    # If MotorU8 isn't present, fall back to a stable constant derived from
    # the official config (Kv=100 rpm/V, Imax=24 A → τ_peak ≈ 2.29 Nm).
    return _kv_to_kt(100.0) * 24.0


def _load_topology(name: str) -> Optional[GearboxType]:
    """Load one (SSPG / CPG / WPG / DSPG) from bundled params + MAD CSV."""
    name_l = name.lower()
    params_path = Path(DEFAULT_GB_PARAMS_TEMPLATE.format(name=name_l))
    csv_path    = Path(DEFAULT_MAD_CSV_TEMPLATE.format(NAME=name.upper()))
    if not params_path.exists() or not csv_path.exists():
        return None
    params = json.loads(params_path.read_text())
    key = f"{name_l}_optimization_parameters"
    opt = params.get(key, {})
    return GearboxType(
        name=name.upper(),
        ratio_min=float(opt.get("GEAR_RATIO_MIN", 4)),
        ratio_max=float(opt.get("GEAR_RATIO_MAX", 60)),
        ratio_step=float(opt.get("GEAR_RATIO_STEP", 1)),
        mad_rows=tuple(load_mad_csv(csv_path)),
        ref_motor_tau_peak=_ref_motor_tau_peak(MOTOR_CATALOG),
    )


def _load_default_gearbox_catalog() -> List[GearboxType]:
    gbs: List[GearboxType] = []
    for n in ("SSPG", "CPG", "WPG", "DSPG"):
        gb = _load_topology(n)
        if gb is not None and gb.mad_rows:
            gbs.append(gb)
    return gbs


GEARBOX_CATALOG: List[GearboxType] = _load_default_gearbox_catalog()


# ---------------------------------------------------------------------------
# Composed actuator (joint-side view)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Actuator:
    """A motor + gearbox bundle, exposing joint-side τ / ω / mass / cost.

    The gearbox-side numbers come from the published COMPAct MAD row chosen
    for ``ratio``; the *peak torque* is then scaled to whatever motor is
    actually bolted on (the published row corresponds to MotorU8).
    """

    name: str
    motor: Motor
    gearbox: GearboxType
    ratio: float
    # The MAD row we resolved to (mass, eff, width, peak_torque_ref).
    mad: MADRow

    # ---- joint-side derived properties (used by the rest of the codebase) ----
    @property
    def eta_gb(self) -> float:
        return self.mad.eff

    @property
    def peak_torque(self) -> float:
        """Continuous joint-side torque cap [Nm].

        We scale the published MAD ``peakTorque`` (computed for the reference
        motor) by the actual motor's τ_peak.  This keeps the dependence on
        ratio / efficiency consistent with COMPAct while honouring our motor.
        """
        scale = self.motor.tau_peak_motor / max(self.gearbox.ref_motor_tau_peak, 1e-6)
        return float(self.mad.peak_torque_Nm * scale)

    @property
    def peak_speed(self) -> float:
        """No-load joint-side speed [rad/s]."""
        return self.motor.omega_nl / self.ratio

    @property
    def mass(self) -> float:
        # Gearbox mass scales mildly with the torque it must carry. We use a
        # square-root scaling for the gearbox portion when the motor is
        # heavier/lighter than the published U8 reference.
        ref_tau = self.gearbox.ref_motor_tau_peak
        torque_scale = math.sqrt(max(self.motor.tau_peak_motor / max(ref_tau, 1e-6), 1e-3))
        return float(self.motor.mass + self.mad.mass_kg * torque_scale)

    @property
    def cost(self) -> float:
        # COMPAct's "Cost" column is dimensionless.  Convert via the same
        # $/kg surrogate we use for motors, applied to gearbox mass.
        gearbox_cost = _cost_from_mass(self.mad.mass_kg)
        return float(self.motor.cost + gearbox_cost)

    @property
    def diameter(self) -> float:
        """Outer housing diameter [m]: the wider of the motor body and the
        gearbox stage, since the assembled actuator can't be narrower than
        either part."""
        motor_d = self.motor.diameter if self.motor.diameter is not None else 0.0
        return max(motor_d, self.mad.width_m)

    @property
    def length(self) -> float:
        """Axial housing thickness [m]. The published MAD rows only carry a
        gearbox *diameter*, not its axial length, so this uses the motor's
        own measured body length as the best available lower bound (a
        gearbox stage mounts behind the motor and adds to this, but that
        isn't in the published data). Falls back to the 0.6*diameter
        heuristic used elsewhere in this file when the motor's length is
        unknown too."""
        if self.motor.length is not None:
            return self.motor.length
        return 0.6 * self.diameter

    # ---- electrical-power model -------------------------------------------
    def electrical_power(self, tau_joint: float, omega_joint: float) -> float:
        """Instantaneous electrical input power [W] given joint-side τ and ω.

        Direction-aware (motoring vs braking); no regen credit.
        """
        N   = self.ratio
        eta = max(self.eta_gb, 1e-3)
        Kt  = self.motor.Kt
        R   = self.motor.R

        motoring = (tau_joint * omega_joint) > 0.0
        if motoring:
            tau_m = tau_joint / (N * eta)
        else:
            tau_m = tau_joint * eta / N
        omega_m = omega_joint * N

        I = tau_m / Kt
        P_mech_m = omega_m * tau_m
        P_cu     = I * I * R
        return float(max(P_mech_m + P_cu, 0.0)) if motoring \
               else float(max(P_cu, 0.0))


# ---------------------------------------------------------------------------
# Catalog builder
# ---------------------------------------------------------------------------
def _subsample_rows(rows: Tuple[MADRow, ...], n: int) -> List[MADRow]:
    """Pick ``n`` rows evenly spaced (by index) across the available ratios.

    Honouring the discrete published ratios — *no interpolation*. If fewer
    rows are available than requested, all of them are returned.
    """
    if not rows:
        return []
    if n >= len(rows):
        return list(rows)
    idxs = [round(i * (len(rows) - 1) / max(n - 1, 1)) for i in range(n)]
    # de-dup while preserving order
    seen, out = set(), []
    for i in idxs:
        if i not in seen:
            seen.add(i)
            out.append(rows[i])
    return out


def build_compact_catalog(
    motors:    List[Motor]       = None,
    gearboxes: List[GearboxType] = None,
    ratios_per_type: int = 3,
) -> List["Actuator"]:
    """Cross-product of motors × gearbox topologies × *published* ratios.

    Every entry corresponds to a tooth-count-feasible design from the
    official COMPAct MAD CSVs — no made-up ratios.
    """
    if motors is None:
        motors = MOTOR_CATALOG
    if gearboxes is None:
        gearboxes = GEARBOX_CATALOG
    out: List[Actuator] = []
    for m in motors:
        for gb in gearboxes:
            for row in _subsample_rows(gb.mad_rows, ratios_per_type):
                # Refresh gearbox.ref_motor_tau_peak in case MOTOR_CATALOG was
                # reloaded from a custom JSON; the field on the frozen
                # GearboxType is fixed at load time, but our Actuator scales
                # via `self.gearbox.ref_motor_tau_peak` so it stays consistent.
                out.append(Actuator(
                    name=f"{m.name}+{gb.name}:N={row.ratio:.2f}",
                    motor=m, gearbox=gb, ratio=row.ratio, mad=row,
                ))
    return out


# ---------------------------------------------------------------------------
# Catalog-mode actuator: a flat, pre-built actuator (real legged-robot units).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CatalogActuator:
    """A pre-built actuator entry (no decomposition into motor+gearbox).

    Used when the outer loop should *pick from a fixed shelf* of real
    actuators (Unitree A1, ANYdrive, AK80-9, etc.) instead of designing one
    via COMPAct. Exposes the same joint-side interface as :class:`Actuator`
    so downstream code (evaluator, spec, controller) is unaware of the mode.
    """

    name: str
    peak_torque: float          # [N·m] joint side
    peak_speed:  float          # [rad/s] joint side
    mass: float                 # [kg]
    cost: float                 # [$]
    eta_gb: float = 0.85        # overall transmission efficiency
    diameter: float = 0.080     # outer diameter / housing width [m]
    length: float = 0.048       # axial thickness / housing length along its rotation
                                 # axis [m] -- NOT in the original catalog schema (which
                                 # only tracked outer diameter). Needed for the x/y/z
                                 # placement formulas in builder.py, which treat radius
                                 # and axial half-length as independent. When a JSON
                                 # entry doesn't specify "length", the loader below
                                 # defaults it to 0.6 * diameter (a flat-pancake-BLDC
                                 # heuristic) -- flagged there, not a measured value.
    # Optional electrical params for a high-fidelity power model.
    Kt:    Optional[float] = None     # motor torque constant [N·m/A]
    R:     Optional[float] = None     # phase resistance [Ω]
    ratio: Optional[float] = None     # gearbox ratio
    notes: str = ""

    # ---- electrical-power model ------------------------------------------
    def electrical_power(self, tau_joint: float, omega_joint: float) -> float:
        """Direction-aware electrical input power [W].

        If ``Kt`` / ``R`` / ``ratio`` are provided, uses the same motor-side
        copper-loss model as :class:`Actuator`. Otherwise falls back to an
        efficiency-only estimate ``P = max(0, τ·ω/η)`` (motoring) and
        ``P = 0`` (braking, no regen credit).
        """
        motoring = (tau_joint * omega_joint) > 0.0
        if self.Kt is not None and self.R is not None and self.ratio is not None:
            N, eta, Kt, R = self.ratio, self.eta_gb, self.Kt, self.R
            if motoring:
                tau_m = tau_joint / (N * eta)
            else:
                tau_m = tau_joint * eta / N
            omega_m = omega_joint * N
            I = tau_m / Kt
            P_cu = I * I * R
            if motoring:
                return float(max(omega_m * tau_m + P_cu, 0.0))
            return float(max(P_cu, 0.0))
        # Fallback: efficiency-only model.
        P_mech = tau_joint * omega_joint
        if motoring:
            return float(P_mech / max(self.eta_gb, 1e-3))
        return 0.0


# Type alias for "anything the outer loop can pick" — duck-typed in callers.
ActuatorLike = Union[Actuator, CatalogActuator]


def load_actuator_catalog_json(path: Union[str, Path]) -> List[CatalogActuator]:
    """Load a flat actuator catalog from JSON (see catalogs/actuators.json).

    Required per entry: ``name``, ``peak_torque``, ``peak_speed``, ``mass``,
    ``cost``. Optional: ``eta``, ``Kt``, ``R``, ``ratio``, ``diameter``,
    ``length``, ``notes``. ``length`` (axial housing thickness) defaults to
    ``0.6 * diameter`` when a JSON entry doesn't specify it -- an unvalidated
    flat-pancake-motor heuristic, not measured data; entries that DO give a
    real ``length`` (e.g. from a datasheet) should always specify it explicitly.
    """
    data = json.loads(Path(path).read_text())
    items = data.get("actuators", data) if isinstance(data, dict) else data
    out: List[CatalogActuator] = []
    for d in items:
        diameter = float(d.get("diameter", 0.080))
        out.append(CatalogActuator(
            name=str(d["name"]),
            peak_torque=float(d["peak_torque"]),
            peak_speed=float(d["peak_speed"]),
            mass=float(d["mass"]),
            cost=float(d["cost"]),
            eta_gb=float(d.get("eta", 0.85)),
            diameter=diameter,
            length=float(d.get("length", 0.6 * diameter)),
            Kt=(float(d["Kt"]) if "Kt" in d else None),
            R=(float(d["R"])  if "R"  in d else None),
            ratio=(float(d["ratio"]) if "ratio" in d else None),
            notes=str(d.get("notes", "")),
        ))
    return out


# Default catalog used by the rest of the codebase. PER REQUEST: actuator
# data is retrieved from the JSON catalog files (catalogs/actuators.json),
# not the COMPAct compact/ artifacts -- those files were never bundled here,
# so the old default (`build_compact_catalog()`) silently resolved to an
# EMPTY catalog (GEARBOX_CATALOG == [] with the compact/ folder missing).
# `set_active_catalog("compact", ...)` is still available if that data
# shows up later.
ACTUATOR_CATALOG: List["ActuatorLike"] = load_actuator_catalog_json(DEFAULT_ACTUATORS_JSON)

# Name -> actuator lookup, built alongside the catalog so callers (spec.py)
# can resolve an actuator by its catalog name instead of a fragile numeric
# index -- index positions shift any time the JSON file is edited; names don't.
ACTUATOR_BY_NAME: Dict[str, "ActuatorLike"] = {a.name: a for a in ACTUATOR_CATALOG}


# ---------------------------------------------------------------------------
# Mode switch
# ---------------------------------------------------------------------------
def set_active_catalog(
    mode: str,
    actuators_json: Optional[Union[str, Path]] = None,
    motors_json:    Optional[Union[str, Path]] = None,
    ratios_per_type: int = 3,
) -> List["ActuatorLike"]:
    """Swap the module-level ``ACTUATOR_CATALOG`` between the two modes.

    Parameters
    ----------
    mode : ``"catalog"`` | ``"compact"``
        - ``"catalog"`` : pre-built actuators loaded from ``actuators_json``
          (default: bundled ``design/catalogs/actuators.json``).
        - ``"compact"`` : motor ⊗ gearbox ⊗ ratio enumeration following
          COMPAct (Singh et al., ICRA 2026). Motors come from
          ``motors_json`` (default: bundled ``design/catalogs/motors.json``).

    Notes
    -----
    Callers (``spec.py``, ``optimize_design.py``) read
    ``design.actuator.ACTUATOR_CATALOG`` lazily so this rebinding takes
    effect immediately for subsequent design instantiations.
    """
    global ACTUATOR_CATALOG, MOTOR_CATALOG, GEARBOX_CATALOG, ACTUATOR_BY_NAME
    if mode == "catalog":
        path = Path(actuators_json) if actuators_json else DEFAULT_ACTUATORS_JSON
        ACTUATOR_CATALOG = load_actuator_catalog_json(path)
    elif mode == "compact":
        if motors_json is not None:
            MOTOR_CATALOG = load_motor_catalog_json(motors_json)
        # Rebuild gearbox catalog so its `ref_motor_tau_peak` reflects the
        # (possibly swapped) motor list.
        GEARBOX_CATALOG = _load_default_gearbox_catalog()
        ACTUATOR_CATALOG = build_compact_catalog(
            motors=MOTOR_CATALOG,
            gearboxes=GEARBOX_CATALOG,
            ratios_per_type=ratios_per_type,
        )
    else:
        raise ValueError(
            f"Unknown actuator mode {mode!r}; expected 'catalog' or 'compact'."
        )
    ACTUATOR_BY_NAME = {a.name: a for a in ACTUATOR_CATALOG}
    return ACTUATOR_CATALOG
