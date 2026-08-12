"""Parametric biped MJCF builder -- matches the body/joint/class layout of
``models/tron/biped.xml`` (the SF TRON1 biped).

Each leg is built as a chain of four bodies, matching ``biped.xml``:

    base_Link -> abad_{SIDE}_Link -> hip_{SIDE}_Link -> knee_{SIDE}_Link -> ankle_{SIDE}_Link

with the same ``<default>`` classes (``abad`` / ``hip`` / ``knee`` / ``ankle``),
the same joint/actuator naming (``{joint}_{SIDE}_Joint`` / ``{joint}_{SIDE}_motor``),
the same sensor set, a ``<freejoint name="root"/>`` on ``base_Link``, and a
single ``home`` keyframe -- so a generated file is a drop-in replacement for
``models/tron/biped.xml`` and needs no changes to
``sim/src/models/TronBipedSpec.cpp`` or ``config/tron/my_controller.yaml``
(besides pointing ``model.xml_path`` at it).

This is a direct port of quadruped_xml_builder's ``builder.py``: same
density/volume mass model, same cylinder/box inertia helpers, same 2-link
home-pose IK, same f-string assembly style. What changed for the biped:

  - 2 legs (``R``/``L``), not 4 -- mirrored only in Y, not X and Y.
  - 4 joints per leg (ab/ad, hip, knee, ankle), not 3 -- the extra ankle
    joint's home angle is solved as ``-(hip_home + knee_home)`` so the foot
    stays flat, which is exact here because ab/ad is the only joint whose
    axis isn't Y (see ``home_leg_angles`` below).
  - Actuators are ``<motor>`` (direct torque control), not ``<position>`` --
    matching ``biped.xml`` and what ``My_Controller`` actually commands.
  - Proximal actuator mounting, matching biped.xml exactly: every actuator's
    housing/mass sits one body *up* the chain from the joint it drives, not
    co-located with its own joint. Verified by rotating each housing
    cylinder's local +Z axis through its own ``quat`` in ``biped.xml``: every
    actuator-style cylinder maps to world/parent -Y (the hip/knee/ankle
    rotation axis, preserved through the chain since every inter-body
    pre-bend quat is a pure Y rotation) *except* the one static, unjointed
    bracket geom sitting directly in ``base_Link`` before ``abad_{SIDE}_Link``,
    which maps to +X (the ab/ad axis). So:
      - the ab/ad actuator's own housing is a static, massless-body decal on
        ``base_Link`` (its mass is folded into ``base_Link``'s own inertial,
        see ``build_xml``) -- there is nothing further up the chain to mount
        it on;
      - the *hip* actuator's housing/mass sits on ``abad_{SIDE}_Link``;
      - the *knee* actuator's housing/mass sits on ``hip_{SIDE}_Link``;
      - the *ankle* actuator's housing/mass sits on ``knee_{SIDE}_Link``;
      - ``ankle_{SIDE}_Link`` itself carries no actuator at all -- just foot
        structure, matching biped.xml (0.7 kg, no housing geom).
    Each joint still gets its own ``<default>``/``<motor>`` torque limit from
    its own ``DesignSpec`` actuator pick (``act_abad`` sets the ``abad``
    class's forcerange, etc.) -- ``build_xml`` computes those straight from
    ``spec.actuators()``, independent of where each actuator's mass/housing
    geometry lives.
  - Leg links are straight-down-local-Z chains like the quadruped's
    thigh/calf (no CAD-style pre-bend quats) -- simpler and general, but
    means a generated file is only *structurally* similar to biped.xml, not
    byte-identical. See GEOMETRY_RULES.md for the fixed hardware constants
    (housing radii, strut cross-sections, joint dynamics/ranges) this
    builder uses in place of biped.xml's hand-tuned CAD offsets.
"""
from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

import numpy as np

from spec import DesignSpec

# ---------------------------------------------------------------------------
# Leg naming/signs, inlined the same way quadruped_xml_builder's builder.py
# inlines LEG_NAMES/LEG_SIGNS so this file has no dependency on anything else.
LEG_SIDES = ("R", "L")
SIDE_SIGN = {"R": -1.0, "L": 1.0}   # multiplies lateral (y) offsets and the ab/ad axis

# ---------------------------------------------------------------------------
# Fixed actuator-housing geometry (hardware constants, independent of
# DesignSpec -- same role as quadruped_xml_builder's ABD_RADIUS/HIP_RADIUS/
# KNEE_RADIUS). Values match the housing cylinders in biped.xml. Every name
# below is tagged with which joint's actuator it represents and which body it
# physically sits on -- these are one body apart (see module docstring).
ABAD_OWN_HOUSING_RADIUS  = 0.0483   # [m] ab/ad actuator's own housing, decal on base_Link
ABAD_OWN_HOUSING_HALFLEN = 0.02725  # [m]

HIP_AT_ABAD_HOUSING_RADIUS  = 0.0483   # [m] hip actuator, proximally mounted on abad_{SIDE}_Link
HIP_AT_ABAD_HOUSING_HALFLEN = 0.02725  # [m]

KNEE_AT_HIP_HOUSING_RADIUS  = 0.0483   # [m] knee actuator, proximally mounted on hip_{SIDE}_Link
KNEE_AT_HIP_HOUSING_HALFLEN = 0.016    # [m]

ANKLE_AT_KNEE_HOUSING_RADIUS  = 0.03     # [m] ankle actuator, proximally mounted on knee_{SIDE}_Link
ANKLE_AT_KNEE_HOUSING_HALFLEN = 0.02725  # [m]

# Gap kept between two adjacent housings (or a housing and the distal end of
# the body it sits on), so cylinders don't overlap/poke through each other.
HOUSING_GAP = 0.005   # [m]

# hip_{SIDE}_Link's own origin coincides exactly with where the hip actuator
# (housed one body up, in abad_{SIDE}_Link) is drawn -- both sit at
# ABAD_TO_HIP_OFFSET. Since both housings are Y-oriented (coaxial), biped.xml
# separates them by offsetting along their shared long axis (Y), not
# perpendicular to it -- its own two knee-actuator cylinders sit 0.04-0.08 m
# out along Y from hip_R_Link's origin. This offset guarantees the knee
# housing and the hip housing above it don't overlap along that shared axis.
KNEE_AT_HIP_Y_OFFSET = HIP_AT_ABAD_HOUSING_HALFLEN + KNEE_AT_HIP_HOUSING_HALFLEN + HOUSING_GAP

# Fixed strut cross-sections (hollow composite link shells), half-extents.
THIGH_STRUT_HALF_WIDTH = 0.025   # [m]
THIGH_STRUT_HALF_THICK = 0.016   # [m]
SHANK_STRUT_HALF_WIDTH = 0.0125  # [m]
SHANK_STRUT_HALF_THICK = 0.016   # [m]
STRUT_DENSITY = 700.0            # [kg/m^3] effective density of a hollow carbon/AL strut

# Effective trunk density, calibrated so the baseline trunk box
# (0.27 x 0.26 x 0.19 m, matching biped.xml's base_Link) masses ~12 kg
# *before* the ab/ad actuators are folded in (see build_xml).
TRUNK_DENSITY = 899.5   # [kg/m^3]

JOINT_STRUCT_MASS = 0.10   # [kg] mount/bearing structural mass added at each actuator housing
FOOT_STRUCT_MASS  = 0.30   # [kg] foot/ankle bracket mass -- ankle_{SIDE}_Link carries no
                            # actuator of its own (its actuator is mounted on knee_{SIDE}_Link),
                            # so this alone stands in for biped.xml's ankle_{SIDE}_Link mass.

# Fixed mount-bracket offset between the ab/ad joint and the hip joint -- a
# hardware constant, not spec-controlled, matching biped.xml's
# hip_{SIDE}_Link pos relative to abad_{SIDE}_Link. This doubles as the
# position of the hip actuator's housing within abad_{SIDE}_Link (biped.xml
# places that housing exactly here too -- right at the distal end of
# abad_{SIDE}_Link, where hip_{SIDE}_Link attaches).
ABAD_TO_HIP_OFFSET_X = -0.0683   # [m]
ABAD_TO_HIP_OFFSET_Y = 0.0305    # [m], mirrored by SIDE_SIGN

# Foot box placement relative to the ankle joint.
FOOT_X_OFFSET_FRAC = 0.05   # fraction of foot_length the foot box center sits fwd of the ankle
FOOT_ANKLE_GAP = 0.02       # [m] vertical gap from the ankle joint down to the top of the foot box

# Joint dynamics per class -- fixed, matches the <default> classes in biped.xml.
JOINT_DYNAMICS: Dict[str, Dict[str, float]] = {
    "abad":  {"damping": 1.0, "armature": 0.05, "frictionloss": 0.10},
    "hip":   {"damping": 1.0, "armature": 0.05, "frictionloss": 0.10},
    "knee":  {"damping": 1.0, "armature": 0.05, "frictionloss": 0.10},
    "ankle": {"damping": 0.5, "armature": 0.02, "frictionloss": 0.01},
}

# Joint ranges per class [rad] -- one canonical range per joint type, mirrored
# across sides. (biped.xml's real ranges are almost, but not exactly,
# symmetric between R/L -- this generator uses one canonical range for both,
# see GEOMETRY_RULES.md.)
JOINT_RANGES: Dict[str, Tuple[float, float]] = {
    "abad":  (-0.38397, 1.39626),
    "hip":   (-1.39626, 1.01229),
    "knee":  (-1.21357, 1.17267),
    "ankle": (-0.791917, 1.30899),
}

# Minimum ab/ad joint half-spacing so the two ab/ad housing decals (and their
# mount brackets) don't collide across the trunk centerline.
MIN_HIP_Y = ABAD_OWN_HOUSING_RADIUS + 0.01


def home_leg_angles(L_thigh: float, L_shank: float, body_height: float) -> Tuple[float, float]:
    """2-link IK for a foot directly below the hip at depth ``body_height``.

    Identical formula to quadruped_xml_builder's ``home_leg_angles`` (law of
    cosines); the biped's extra ankle joint is not part of this solve -- see
    ``build_xml`` for how the ankle home angle is derived from these two.
    """
    h = float(body_height)
    L1, L2 = float(L_thigh), float(L_shank)
    h = float(np.clip(h, abs(L1 - L2) + 1e-3, L1 + L2 - 1e-3))
    a = (L1 * L1 + h * h - L2 * L2) / (2.0 * L1 * h)
    b = (L1 * L1 + L2 * L2 - h * h) / (2.0 * L1 * L2)
    a = float(np.clip(a, -1.0, 1.0))
    b = float(np.clip(b, -1.0, 1.0))
    theta_hip = math.acos(a)
    theta_knee = -math.acos(-b)
    return theta_hip, theta_knee


# ---------------------------------------------------------------------------
# Inertia helpers (unchanged from quadruped_xml_builder -- generic rigid-body
# physics, not leg-count-specific).
# ---------------------------------------------------------------------------
def _box_inertia(mass: float, lx: float, ly: float, lz: float) -> Tuple[float, float, float]:
    Ixx = mass * (ly * ly + lz * lz) / 12.0
    Iyy = mass * (lx * lx + lz * lz) / 12.0
    Izz = mass * (lx * lx + ly * ly) / 12.0
    return Ixx, Iyy, Izz


def _cylinder_inertia(mass: float, radius: float, half_length: float) -> Tuple[float, float, float]:
    """Diagonal inertia of a solid cylinder about its own central axis.

    Returns ``(I_axial, I_radial, I_radial)`` -- caller reorders into
    (Ixx, Iyy, Izz) depending on which world/body axis the cylinder's own
    axis is rotated onto: ``(I_axial, I_radial, I_radial)`` if rotated onto
    X (``quat="1 0 1 0"``, the ab/ad axis), or ``(I_radial, I_axial,
    I_radial)`` if rotated onto Y (``quat="1 1 0 0"``, the hip/knee/ankle
    axis) -- see the module docstring for how this was verified against
    biped.xml's actual quats.
    """
    L = 2.0 * half_length
    I_axial = 0.5 * mass * radius * radius
    I_radial = mass * (3.0 * radius * radius + L * L) / 12.0
    return I_axial, I_radial, I_radial


def _combine_diag_parts(
    parts: Sequence[Tuple[float, Tuple[float, float, float], float]]
) -> Tuple[float, float, Tuple[float, float, float]]:
    """Combine several axisymmetric-about-Z parts, stacked along the shared
    local Z axis with no x/y offset, into one mass/COM/diagonal-inertia triple.

    ``parts`` is ``[(mass, (Ixx,Iyy,Izz) about its own COM, z offset of its
    own COM), ...]``. Because every part here is centered on the z-axis, no
    cross-inertia terms appear and a per-axis parallel-axis sum is exact:
    shifting a part by ``dz`` along z leaves its own Izz unchanged (the shift
    is along that axis) and adds ``mass * dz**2`` to Ixx/Iyy.
    """
    total_mass = sum(m for m, _, _ in parts)
    com_z = sum(m * z for m, _, z in parts) / total_mass
    Ixx = Iyy = Izz = 0.0
    for m, (ixx, iyy, izz), z in parts:
        dz = z - com_z
        Ixx += ixx + m * dz * dz
        Iyy += iyy + m * dz * dz
        Izz += izz
    return total_mass, com_z, (Ixx, Iyy, Izz)


def _fmt(x: float, p: int = 6) -> str:
    return f"{float(x):.{p}f}"


def _vec(*xs) -> str:
    return " ".join(_fmt(x) for x in xs)


# ---------------------------------------------------------------------------
# Geometry rules (see GEOMETRY_RULES.md)
# ---------------------------------------------------------------------------
def check_geometry(spec: DesignSpec) -> None:
    if spec.hip_y < MIN_HIP_Y:
        raise ValueError(
            f"hip_y={spec.hip_y:.4f} m violates Rule 1 in GEOMETRY_RULES.md: "
            f"hip_y must be >= ABAD_OWN_HOUSING_RADIUS + 0.01 ({MIN_HIP_Y:.4f} m) "
            "so the left/right ab/ad housing decals don't collide across the trunk centerline."
        )
    reach = spec.thigh_length + spec.shank_length
    slack = abs(spec.thigh_length - spec.shank_length)
    if not (slack + 1e-3 < spec.body_height < reach - 1e-3):
        print(
            f"[builder] body_height={spec.body_height:.4f} m is outside the reachable "
            f"band ({slack + 1e-3:.4f}, {reach - 1e-3:.4f}) m for thigh={spec.thigh_length:.4f}, "
            f"shank={spec.shank_length:.4f} -- home_leg_angles will clip it (Rule 2)."
        )


# ---------------------------------------------------------------------------
# Per-body MJCF fragments
# ---------------------------------------------------------------------------
def _abad_own_housing_geom(side: str, spec: DesignSpec) -> str:
    """Static, unjointed decal on base_Link representing the ab/ad
    actuator's own housing -- oriented along X (the ab/ad axis, quat
    "1 0 1 0"), matching biped.xml's bracket geom that sits directly in
    base_Link just before abad_{SIDE}_Link (no joint, no separate body: its
    mass is folded into base_Link's own inertial in build_xml, since it
    doesn't move relative to the trunk). Visual only -- biped.xml doesn't
    give this one a collision copy either.
    """
    sign = SIDE_SIGN[side]
    x = spec.hip_x + ABAD_OWN_HOUSING_HALFLEN
    return (f'      <geom class="visual" type="cylinder" '
            f'size="{_vec(ABAD_OWN_HOUSING_RADIUS, ABAD_OWN_HOUSING_HALFLEN)}" '
            f'pos="{_fmt(x)} {_fmt(sign * spec.hip_y)} {_fmt(-spec.hip_z_drop)}" quat="1 0 1 0"/>')


def _abad_body(side: str, spec: DesignSpec, act_hip, inner_xml: str) -> str:
    """abad_{SIDE}_Link: carries the ab/ad joint itself, but its housing/mass
    is the *hip* actuator (proximally mounted here, driving hip_{SIDE}_Joint
    one body down through abad_{SIDE}_Link -> hip_{SIDE}_Link). Positioned at
    ABAD_TO_HIP_OFFSET -- the distal end of this body, right where
    hip_{SIDE}_Link attaches -- matching biped.xml exactly."""
    sign = SIDE_SIGN[side]
    mass = act_hip.mass + JOINT_STRUCT_MASS
    I_ax, I_r, _ = _cylinder_inertia(mass, HIP_AT_ABAD_HOUSING_RADIUS, HIP_AT_ABAD_HOUSING_HALFLEN)
    # quat="1 1 0 0" swings the cylinder's local z axis onto body/world y (the
    # hip joint's rotation axis -- this is the HIP actuator, not ab/ad's own).
    lo, hi = JOINT_RANGES["abad"]
    hx, hy = ABAD_TO_HIP_OFFSET_X, sign * ABAD_TO_HIP_OFFSET_Y
    return f"""
    <body name="abad_{side}_Link" pos="{_fmt(spec.hip_x)} {_fmt(sign * spec.hip_y)} {_fmt(-spec.hip_z_drop)}">
      <inertial pos="{_fmt(hx)} {_fmt(hy)} 0" mass="{_fmt(mass)}" diaginertia="{_vec(I_r, I_ax, I_r)}"/>
      <joint class="abad" name="abad_{side}_Joint" axis="{_fmt(sign)} 0 0" range="{_fmt(lo)} {_fmt(hi)}"/>
      <geom class="visual" type="cylinder" size="{_vec(HIP_AT_ABAD_HOUSING_RADIUS, HIP_AT_ABAD_HOUSING_HALFLEN)}" pos="{_fmt(hx)} {_fmt(hy)} 0" quat="1 1 0 0"/>
      <geom class="collision" type="cylinder" size="{_vec(HIP_AT_ABAD_HOUSING_RADIUS, HIP_AT_ABAD_HOUSING_HALFLEN)}" pos="{_fmt(hx)} {_fmt(hy)} 0" quat="1 1 0 0"/>
{inner_xml}
    </body>"""


def _hip_body(side: str, spec: DesignSpec, act_knee, inner_xml: str) -> str:
    """hip_{SIDE}_Link: carries the hip joint, but its housing/mass is the
    *knee* actuator (proximally mounted here, driving knee_{SIDE}_Joint one
    body down). Positioned near this body's *proximal* origin -- clustered
    together with the ab/ad decal and the hip actuator -- not out near the
    knee joint. hip_{SIDE}_Link's own origin coincides exactly with the hip
    actuator's position (both sit at ABAD_TO_HIP_OFFSET), so the knee
    housing is offset from that origin along Y (its own long axis, since
    both housings are coaxial Y-oriented cylinders) by
    KNEE_AT_HIP_Y_OFFSET -- matching biped.xml, which staggers its own two
    knee-actuator cylinders along Y from hip_R_Link's origin (0.04-0.08 m,
    not along Z) to keep them from overlapping the coaxial hip housing right
    next to them. The thigh strut still spans the full reach straight down
    to knee_{SIDE}_Link regardless of where this housing decal sits."""
    sign = SIDE_SIGN[side]
    L1 = spec.thigh_length
    mass_knee = act_knee.mass + JOINT_STRUCT_MASS
    strut_mass = STRUT_DENSITY * (2 * THIGH_STRUT_HALF_WIDTH) * (2 * THIGH_STRUT_HALF_THICK) * L1
    I_ax, I_r, _ = _cylinder_inertia(mass_knee, KNEE_AT_HIP_HOUSING_RADIUS, KNEE_AT_HIP_HOUSING_HALFLEN)
    strut_I = _box_inertia(strut_mass, 2 * THIGH_STRUT_HALF_WIDTH, 2 * THIGH_STRUT_HALF_THICK, L1)
    # quat="1 1 0 0" swings local z onto body/world y (knee axis) -> (radial, axial, radial).
    housing_y = sign * KNEE_AT_HIP_Y_OFFSET
    total_mass, com_z, diag = _combine_diag_parts([
        (mass_knee, (I_r, I_ax, I_r), 0.0),
        (strut_mass, strut_I, -L1 / 2.0),
    ])
    com_y = mass_knee * housing_y / total_mass  # strut contributes y=0
    lo, hi = JOINT_RANGES["hip"]
    return f"""
        <body name="hip_{side}_Link" pos="{_fmt(ABAD_TO_HIP_OFFSET_X)} {_fmt(sign * ABAD_TO_HIP_OFFSET_Y)} 0">
          <inertial pos="0 {_fmt(com_y)} {_fmt(com_z)}" mass="{_fmt(total_mass)}" diaginertia="{_vec(*diag)}"/>
          <joint class="hip" name="hip_{side}_Joint" axis="0 -1 0" range="{_fmt(lo)} {_fmt(hi)}"/>
          <geom class="visual" type="cylinder" size="{_vec(KNEE_AT_HIP_HOUSING_RADIUS, KNEE_AT_HIP_HOUSING_HALFLEN)}" pos="0 {_fmt(housing_y)} 0" quat="1 1 0 0"/>
          <geom class="collision" type="cylinder" size="{_vec(KNEE_AT_HIP_HOUSING_RADIUS, KNEE_AT_HIP_HOUSING_HALFLEN)}" pos="0 {_fmt(housing_y)} 0" quat="1 1 0 0"/>
          <geom class="visual" type="box" size="{_vec(THIGH_STRUT_HALF_WIDTH, THIGH_STRUT_HALF_THICK, L1 / 2.0)}" pos="0 0 {_fmt(-L1 / 2.0)}"/>
          <geom class="collision" type="box" size="{_vec(THIGH_STRUT_HALF_WIDTH, THIGH_STRUT_HALF_THICK, L1 / 2.0)}" pos="0 0 {_fmt(-L1 / 2.0)}"/>
{inner_xml}
        </body>"""


def _knee_body(side: str, spec: DesignSpec, act_ankle, inner_xml: str) -> str:
    """knee_{SIDE}_Link: carries the knee joint, but its housing/mass is the
    *ankle* actuator (proximally mounted here, driving ankle_{SIDE}_Joint one
    body down), positioned near this body's distal end -- close to
    ankle_{SIDE}_Link's attachment point -- plus the shank strut."""
    L2 = spec.shank_length
    mass_ankle = act_ankle.mass + JOINT_STRUCT_MASS
    strut_mass = STRUT_DENSITY * (2 * SHANK_STRUT_HALF_WIDTH) * (2 * SHANK_STRUT_HALF_THICK) * L2
    I_ax, I_r, _ = _cylinder_inertia(mass_ankle, ANKLE_AT_KNEE_HOUSING_RADIUS, ANKLE_AT_KNEE_HOUSING_HALFLEN)
    strut_I = _box_inertia(strut_mass, 2 * SHANK_STRUT_HALF_WIDTH, 2 * SHANK_STRUT_HALF_THICK, L2)
    housing_z = -L2 + ANKLE_AT_KNEE_HOUSING_HALFLEN + HOUSING_GAP
    total_mass, com_z, diag = _combine_diag_parts([
        (mass_ankle, (I_r, I_ax, I_r), housing_z),
        (strut_mass, strut_I, -L2 / 2.0),
    ])
    lo, hi = JOINT_RANGES["knee"]
    return f"""
            <body name="knee_{side}_Link" pos="0 0 {_fmt(-spec.thigh_length)}">
              <inertial pos="0 0 {_fmt(com_z)}" mass="{_fmt(total_mass)}" diaginertia="{_vec(*diag)}"/>
              <joint class="knee" name="knee_{side}_Joint" axis="0 -1 0" range="{_fmt(lo)} {_fmt(hi)}"/>
              <geom class="visual" type="box" size="{_vec(SHANK_STRUT_HALF_WIDTH, SHANK_STRUT_HALF_THICK, L2 / 2.0)}" pos="0 0 {_fmt(-L2 / 2.0)}"/>
              <geom class="collision" type="box" size="{_vec(SHANK_STRUT_HALF_WIDTH, SHANK_STRUT_HALF_THICK, L2 / 2.0)}" pos="0 0 {_fmt(-L2 / 2.0)}"/>
              <geom class="visual" type="cylinder" size="{_vec(ANKLE_AT_KNEE_HOUSING_RADIUS, ANKLE_AT_KNEE_HOUSING_HALFLEN)}" pos="0 0 {_fmt(housing_z)}" quat="1 1 0 0"/>
              <geom class="collision" type="cylinder" size="{_vec(ANKLE_AT_KNEE_HOUSING_RADIUS, ANKLE_AT_KNEE_HOUSING_HALFLEN)}" pos="0 0 {_fmt(housing_z)}" quat="1 1 0 0"/>
{inner_xml}
            </body>"""


def _ankle_body(side: str, spec: DesignSpec) -> str:
    """ankle_{SIDE}_Link: just the foot -- its actuator is mounted on
    knee_{SIDE}_Link (see _knee_body), matching biped.xml where this body has
    no housing geom and its mass is foot/bracket structure, not a motor."""
    L2 = spec.shank_length
    mass = FOOT_STRUCT_MASS
    x_off = FOOT_X_OFFSET_FRAC * spec.foot_length
    z_center = -(FOOT_ANKLE_GAP + spec.foot_height / 2.0)
    z_bottom = -(FOOT_ANKLE_GAP + spec.foot_height)
    diag = _box_inertia(mass, spec.foot_length, spec.foot_width, spec.foot_height)
    lo, hi = JOINT_RANGES["ankle"]
    foot_size = _vec(spec.foot_length / 2.0, spec.foot_width / 2.0, spec.foot_height / 2.0)
    return f"""
                <body name="ankle_{side}_Link" pos="0 0 {_fmt(-L2)}">
                  <inertial pos="{_fmt(x_off)} 0 {_fmt(z_center)}" mass="{_fmt(mass)}" diaginertia="{_vec(*diag)}"/>
                  <joint class="ankle" name="ankle_{side}_Joint" axis="0 -1 0" range="{_fmt(lo)} {_fmt(hi)}"/>
                  <geom class="visual" type="box" size="{foot_size}" pos="{_fmt(x_off)} 0 {_fmt(z_center)}"/>
                  <geom class="collision" name="foot_{side}_collision" type="box" size="{foot_size}" pos="{_fmt(x_off)} 0 {_fmt(z_center)}"/>
                  <site name="foot_{side}" pos="{_fmt(x_off)} 0 {_fmt(z_bottom)}" size="0.01"/>
                </body>"""


def _leg_xml(side: str, spec: DesignSpec, act_hip, act_knee, act_ankle) -> Tuple[str, str]:
    """Returns (decal_geom_xml_for_base_Link, abad_body_xml)."""
    ankle_xml = _ankle_body(side, spec)
    knee_xml = _knee_body(side, spec, act_ankle, ankle_xml)
    hip_xml = _hip_body(side, spec, act_knee, knee_xml)
    abad_xml = _abad_body(side, spec, act_hip, hip_xml)
    decal_xml = _abad_own_housing_geom(side, spec)
    return decal_xml, abad_xml


# ---------------------------------------------------------------------------
# Top-level MJCF assembly
# ---------------------------------------------------------------------------
def build_xml(spec: DesignSpec) -> str:
    """Return the full MJCF string for ``spec``, structured to match
    ``models/tron/biped.xml`` (default classes, body/joint/actuator naming,
    sensors, freejoint, single home keyframe)."""
    check_geometry(spec)

    act_abad, act_hip, act_knee, act_ankle = spec.actuators()

    m_trunk_box = TRUNK_DENSITY * spec.trunk_length * spec.trunk_width * spec.trunk_thickness
    trunk_box_I = _box_inertia(m_trunk_box, spec.trunk_length, spec.trunk_width, spec.trunk_thickness)

    # Both ab/ad actuators (one per leg) are folded into base_Link's own
    # inertial, matching biped.xml (their housing is a massless decal, not a
    # separate body -- see _abad_own_housing_geom). Approximation: only the
    # shared z-offset (-hip_z_drop) is tracked via the parallel-axis
    # combiner; the lateral (+/-hip_y) and fore-aft (hip_x) offsets are
    # dropped, the same z-only simplification used throughout this file.
    m_abad_pair = 2.0 * (act_abad.mass + JOINT_STRUCT_MASS)
    I_ax_abad, I_r_abad, _ = _cylinder_inertia(
        act_abad.mass + JOINT_STRUCT_MASS, ABAD_OWN_HOUSING_RADIUS, ABAD_OWN_HOUSING_HALFLEN)
    m_trunk, trunk_com_z, trunk_I = _combine_diag_parts([
        (m_trunk_box, trunk_box_I, 0.0),
        (m_abad_pair, (I_ax_abad, I_r_abad, I_r_abad), -spec.hip_z_drop),
    ])

    theta_hip, theta_knee = home_leg_angles(spec.thigh_length, spec.shank_length, spec.body_height)
    theta_ankle = -(theta_hip + theta_knee)  # keeps the foot flat -- exact here since
    # ab/ad is the only joint axis that differs from Y (see home_leg_angles docstring).

    base_z = spec.hip_z_drop + spec.body_height

    right_decal, right_xml = _leg_xml("R", spec, act_hip, act_knee, act_ankle)
    left_decal, left_xml = _leg_xml("L", spec, act_hip, act_knee, act_ankle)

    qpos_leg = [0.0, theta_hip, theta_knee, theta_ankle]
    qpos = [0.0, 0.0, base_z, 1.0, 0.0, 0.0, 0.0] + qpos_leg * 2
    qpos_str = " ".join(_fmt(q) for q in qpos)

    def _motor(joint: str, side: str) -> str:
        return (f'    <motor class="{joint}" name="{joint}_{side}_motor" '
                f'joint="{joint}_{side}_Joint"/>')

    actuators_xml = "\n".join(
        _motor(joint, side)
        for side in LEG_SIDES
        for joint in ("abad", "hip", "knee", "ankle")
    )

    # Per-joint torque limits come straight from the actuator picked to drive
    # that joint -- independent of which body its housing mass lives on.
    limits = {
        "abad": act_abad.peak_torque,
        "hip": act_hip.peak_torque,
        "knee": act_knee.peak_torque,
        "ankle": act_ankle.peak_torque,
    }

    def _default_class(name: str) -> str:
        dyn = JOINT_DYNAMICS[name]
        lim = _fmt(abs(limits[name]))
        return f"""    <default class="{name}">
      <joint damping="{_fmt(dyn['damping'])}" armature="{_fmt(dyn['armature'])}" frictionloss="{_fmt(dyn['frictionloss'])}"/>
      <motor ctrlrange="-{lim} {lim}" forcerange="-{lim} {lim}"/>
    </default>"""

    # abad/hip/knee/ankle torque limits are shared across sides (one actuator
    # pick per joint *type*, mirrored -- like DesignSpec.act_abduction etc.).
    defaults_xml = "\n".join(_default_class(name) for name in ("abad", "hip", "knee", "ankle"))

    return f"""<mujoco model="design_biped">
  <compiler angle="radian" autolimits="true"/>

  <option timestep="0.002" iterations="50" ls_iterations="50" integrator="implicitfast" cone="elliptic">
    <flag eulerdamp="disable"/>
  </option>

  <visual>
    <headlight ambient="0.3 0.3 0.3" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>
    <global azimuth="120" elevation="-15"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
              rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <default>
    <geom contype="1" conaffinity="0" friction="0.9 0.02 0.001" solimp="0.9 0.95 0.001" solref="0.01 1"/>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="2" rgba="0.7 0.7 0.75 1"/>
    </default>
    <default class="collision">
      <geom contype="1" conaffinity="1" group="3" rgba="0.7 0.5 0.3 0.4"/>
    </default>
{defaults_xml}
  </default>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.05" material="groundplane" friction="0.9 0.02 0.001"
          contype="1" conaffinity="1"/>

    <body name="base_Link" pos="0 0 {_fmt(base_z)}">
      <freejoint name="root"/>
      <inertial pos="0 0 {_fmt(trunk_com_z)}" mass="{_fmt(m_trunk)}" diaginertia="{_vec(*trunk_I)}"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      <geom class="visual" type="box" size="{_vec(spec.trunk_length / 2.0, spec.trunk_width / 2.0, spec.trunk_thickness / 2.0)}" rgba="0.79216 0.81961 0.93333 1"/>
      <geom class="collision" type="box" size="{_vec(spec.trunk_length / 2.0, spec.trunk_width / 2.0, spec.trunk_thickness / 2.0)}"/>
{right_decal}
{right_xml}
{left_decal}
{left_xml}
    </body>
  </worldbody>

  <actuator>
{actuators_xml}
  </actuator>

  <sensor>
    <framequat name="base_quat" objtype="site" objname="imu"/>
    <framepos name="base_pos" objtype="site" objname="imu"/>
    <gyro name="base_gyro" site="imu"/>
    <accelerometer name="base_accel" site="imu"/>
    <framepos name="foot_R_pos" objtype="site" objname="foot_R"/>
    <framepos name="foot_L_pos" objtype="site" objname="foot_L"/>
    <touch name="foot_R_touch" site="foot_R"/>
    <touch name="foot_L_touch" site="foot_L"/>
    <subtreecom name="base_subtreecom" body="base_Link"/>
    <subtreelinvel name="base_subtreelinvel" body="base_Link"/>
  </sensor>

  <keyframe>
    <key name="home" qpos="{qpos_str}"/>
  </keyframe>
</mujoco>
"""
