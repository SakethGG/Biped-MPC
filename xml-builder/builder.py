"""Parametric biped MJCF builder -- matches the ACTUATOR-CLUSTER topology of
``design_biped.xml`` / ``new_biped.xml`` (not ``models/tron/biped.xml``'s own
simple abad_Link->hip_Link->knee_Link->ankle_Link chain -- an earlier version
of this file built that instead; see git history / chat log if you need it).

Each leg is a genuine actuator cluster, chained:

    base_Link -> abad_{SIDE}_act -> hip_{SIDE}_act -> knee_{SIDE}_act
        -> thigh_{SIDE}_Link -> shank_{SIDE}_Link -> ankle_{SIDE}_act
        -> foot_{SIDE}_Link

- ``abad_{SIDE}_act`` / ``hip_{SIDE}_act`` / ``knee_{SIDE}_act`` are real
  actuator-housing bodies (cylinders), positioned by the clearance-based x/y/z
  offset formulas given in chat, chained off each other -- NOT co-located
  with the joint they drive and NOT one-body-up like biped.xml's real
  layout. ``abad_{SIDE}_act`` is welded to ``base_Link`` (no <joint>);
  ``hip_{SIDE}_act`` carries ``abad_{SIDE}_Joint``; ``knee_{SIDE}_act``
  carries ``hip_{SIDE}_Joint``.
- ``thigh_{SIDE}_Link`` is welded to ``knee_{SIDE}_act``: a single cuboid
  (fixed cross-section, ``THIGH_BOX_HALF``) offset from the body's own
  origin by the knee actuator's radius (to avoid overlapping
  ``knee_{SIDE}_act``'s own cylinder geometry), pointing along biped.xml's
  real hip->knee bone direction, scaled to ``spec.thigh_length``.
- ``shank_{SIDE}_Link`` carries the belt-driven ``knee_{SIDE}_Joint`` -- even
  though the physical knee motor's housing/mass sits up in
  ``knee_{SIDE}_act``, the joint DOF itself (and its torque/limit numbers)
  live here, since a belt only relocates where torque is applied
  mechanically, not where MuJoCo needs the joint. Placeholder box geometry
  (not yet spec-controlled -- see ``SHANK_BOX_HALF`` below).
- ``ankle_{SIDE}_act`` is the ankle actuator housing, welded to
  ``shank_{SIDE}_Link`` (no ``<joint>``) -- same treatment as
  abad/hip/knee's housings. ``foot_{SIDE}_Link`` is a separate body one
  level further down, carrying ``ankle_{SIDE}_Joint`` (the actual revolute
  DOF) and the ``foot_{SIDE}`` sensor site.

This topology derives every actuator-cluster/thigh/ankle/foot position from
trunk dimensions + actuator radii + every ``clearance_{source}2{target}_{axis}``
field on ``DesignSpec`` (see its class docstring in generate.py for the
free/fixed-constant split -- both tiers are read here, the fixed-constant
ones just default to 0.0 unless overridden), not from a free
``hip_x``/``hip_y``/``hip_z_drop`` mount offset the way biped.xml's real
chain works -- those three fields are gone from DesignSpec (see
generate.py). ``body_height`` now sets ``base_Link``'s
spawn height directly (no more ``+ hip_z_drop`` composition, and no 2-link
IK for the home pose -- this topology's leg isn't a simple 2-link chain, so
home angles are fixed small values instead, matching design_biped.xml's own
``0, 0.1, 0.15, -0.1`` convention).

Still NOT spec-controlled: the thigh/shank/foot box cross-sections
(``THIGH_BOX_HALF``/``SHANK_BOX_HALF``/``FOOT_BOX_HALF`` are all fixed
constants loaded from ``constant_params.json``, not ``DesignSpec`` fields --
``thigh_width``/``thigh_thickness``/``shank_width``/``shank_thickness`` and
``foot_width``/``foot_height`` used to be spec fields feeding these, but
were removed once nothing else needed them to vary independently).

Every module-level constant this file uses (box cross-sections, bend
angles, joint ranges, densities, joint dynamics, link colors) is loaded
from ``constant_params.json`` at import time -- edit that file, not this
one, to change a constant's value. ``DesignSpec``'s own field values live
in the separate ``variable_params.json`` -- see ``generate.py``.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

CONSTANT_PARAMS_PATH = Path(__file__).parent / "constant_params.json"


def _quat_y(deg: float) -> str:
    """MJCF quat string 'w x y z' for a rotation of `deg` about the Y axis."""
    theta = math.radians(deg) / 2.0
    return f"{math.cos(theta):.6f} 0 {math.sin(theta):.6f} 0"


def _quat_x(deg: float) -> str:
    """MJCF quat string 'w x y z' for a rotation of `deg` about the X axis."""
    theta = math.radians(deg) / 2.0
    return f"{math.cos(theta):.6f} {math.sin(theta):.6f} 0 0"


# ---------------------------------------------------------------------------
# Fixed hardware/CAD constants -- loaded from constant_params.json (see
# GEOMETRY_RULES.md's "Fixed hardware constants" table for what each one
# means). Not part of DesignSpec -- see generate.py/variable_params.json for
# that. To restore the old "shank stays aligned with thigh's frame" no-bend
# experiment, set constant_params.json's SHANK_BEND_DEG to 0.0.
with open(CONSTANT_PARAMS_PATH) as _f:
    _C = json.load(_f)

LEG_SIDES = tuple(_C["LEG_SIDES"])
SIDE_SIGN = _C["SIDE_SIGN"]
LINK_BOX_LENGTH_FRAC = _C["LINK_BOX_LENGTH_FRAC"]
THIGH_BOX_HALF = tuple(_C["THIGH_BOX_HALF"])   # (local-X half-extent, local-Y half-extent)
SHANK_BOX_HALF = tuple(_C["SHANK_BOX_HALF"])   # (half-width, half-thickness) -- cross-section only
FOOT_BOX_HALF = tuple(_C["FOOT_BOX_HALF"])
THIGH_BEND_DEG = _C["THIGH_BEND_DEG"]
SHANK_BEND_DEG = _C["SHANK_BEND_DEG"]          # biped.xml's real thigh->shank bend, fixed
ANKLE_BEND_DEG = _C["ANKLE_BEND_DEG"]

KNEE_RANGE = tuple(_C["KNEE_RANGE"])    # same both legs, matching biped.xml's real knee_R_Joint range
ANKLE_RANGE = tuple(_C["ANKLE_RANGE"])  # same both legs, from biped.xml
ABAD_RANGE = tuple(_C["ABAD_RANGE"])    # same both legs, from biped.xml
HIP_RANGE = tuple(_C["HIP_RANGE"])      # same both legs, from biped.xml

LAMBDA_THIGH = _C["LAMBDA_THIGH"]   # thigh mass-per-length, kg/m -- hardware constant
LAMBDA_SHANK = _C["LAMBDA_SHANK"]   # shank mass-per-length, kg/m -- hardware constant
LAMBDA_FOOT = _C["LAMBDA_FOOT"]     # foot mass-per-length, kg/m -- hardware constant

# trunk shell model: trunk_Link is a hollow enclosure (battery + electronics
# inside), not a solid block -- mass = shell surface area * wall thickness *
# material density, plus a fixed payload mass for whatever's inside. See
# build_xml's trunk mass calc.
TRUNK_SHELL_THICKNESS = _C["TRUNK_SHELL_THICKNESS"]   # shell wall thickness, m
TRUNK_SHELL_DENSITY = _C["TRUNK_SHELL_DENSITY"]       # shell material density, kg/m^3
TRUNK_EXTRA_MASS = _C["TRUNK_EXTRA_MASS"]             # fixed battery/electronics payload mass, kg -- user-supplied

JOINT_DYNAMICS: Dict[str, Dict[str, float]] = _C["JOINT_DYNAMICS"]

# One color per body TYPE (same for both legs), applied to visual geoms only,
# so every link is visually distinguishable in the viewer. Collision geoms
# keep the shared translucent debug rgba from the "collision" default class.
LINK_COLORS: Dict[str, str] = _C["LINK_COLORS"]

# --- derived from the constants above -- NOT themselves stored in
# constant_params.json, so they can't drift out of sync with their inputs --
# biped.xml's real hip->knee and knee->ankle bone directions/bends -- fixed
# hardware/CAD angles; spec.thigh_length/shank_length scale ALONG the
# resulting direction, they don't change it.
SHANK_QUAT = _quat_y(SHANK_BEND_DEG)
ANKLE_QUAT = _quat_y(ANKLE_BEND_DEG)
QUAT_ALIGN_X = _quat_y(_C["QUAT_ALIGN_X_DEG"])   # abad actuator cylinder (axis -> x)
QUAT_ALIGN_Y = _quat_x(_C["QUAT_ALIGN_Y_DEG"])   # hip/knee/ankle actuator cylinder (axis -> y)

_theta_thigh = np.radians(THIGH_BEND_DEG)
THIGH_DIR_XZ = np.array([np.sin(_theta_thigh), np.cos(_theta_thigh)])


def quat_about_y_from_xz_dir(vx: float, vz: float) -> np.ndarray:
    """Quaternion for a pure Y-axis rotation mapping local z-axis (0,0,1) to
    the (normalized) direction (vx, 0, vz)."""
    theta = np.arctan2(vx, vz)
    return np.array([np.cos(theta / 2), 0.0, np.sin(theta / 2), 0.0])


THIGH_BOX_DIR = np.array([THIGH_DIR_XZ[0], 0.0, THIGH_DIR_XZ[1]])
THIGH_BOX_QUAT = quat_about_y_from_xz_dir(THIGH_DIR_XZ[0], THIGH_DIR_XZ[1])


def _linear_density(lam: float, box_half: Tuple[float, float], length_frac: float = 1.0) -> float:
    """Volumetric density (kg/m^3) that makes a fixed-cross-section box's
    MuJoCo-auto-computed mass (density * width * height * geom_length)
    reduce to the linear mass model M = lam * full_length, where full_length
    is the link's real length -- NOT necessarily the box geom's own length.

    `length_frac` is geom_length / full_length: thigh/shank boxes are drawn
    LINK_BOX_LENGTH_FRAC short of their link's real length purely so
    adjacent links don't visually overlap at the joint, so their mass would
    be undercounted by that same fraction unless the density compensates:
    density = mass/(geom_length*B*H) = (lam*full_length)/(length_frac*full_length*B*H)
            = lam/(length_frac*B*H) -- lam, B, H, and length_frac are all
    fixed constants, so this is itself a constant, independent of the link's
    actual length. length_frac=1.0 (the default) is for links -- like the
    foot -- whose box geom already spans their full length."""
    width, height = 2.0 * box_half[0], 2.0 * box_half[1]
    return lam / (width * height * length_frac)


# lam/(B*H*length_frac) per-link -- same linear mass model (mass = lambda *
# full_length), expressed as the equivalent volumetric density MuJoCo's
# density*volume auto-mass needs. thigh/shank correct for their box geom
# being drawn LINK_BOX_LENGTH_FRAC short (see _linear_density) so their mass
# still reflects the link's real, full length, not the shortened box; the
# foot box already spans spec.foot_length in full, so no correction there.
THIGH_DENSITY = _linear_density(LAMBDA_THIGH, THIGH_BOX_HALF, LINK_BOX_LENGTH_FRAC)
SHANK_DENSITY = _linear_density(LAMBDA_SHANK, SHANK_BOX_HALF, LINK_BOX_LENGTH_FRAC)
FOOT_DENSITY = _linear_density(LAMBDA_FOOT, FOOT_BOX_HALF)


# ---------------------------------------------------------------------------
def _housing_dims(act) -> Tuple[float, float]:
    """(radius, halflen) for an actuator's housing cylinder, from its
    catalog ``diameter`` (outer diameter) and ``length`` (axial thickness)."""
    return act.diameter / 2.0, act.length / 2.0


def cyl_inertia(mass: float, r: float, length: float) -> Tuple[float, float]:
    """(I_perp, I_axial) for a solid cylinder about its own centroid, in the
    LOCAL frame where the cylinder's symmetry axis is local z."""
    i_axial = 0.5 * mass * r * r
    i_perp = (1.0 / 12.0) * mass * (3 * r * r + length * length)
    return i_perp, i_axial


def _solid_box_axis_inertia(mass: float, d1: float, d2: float) -> float:
    """Solid-cuboid moment of inertia about one central principal axis,
    given the mass and the box's two OTHER edge lengths (the ones
    perpendicular to that axis) -- the standard (1/12)*m*(d1^2+d2^2)."""
    return (1.0 / 12.0) * mass * (d1 * d1 + d2 * d2)


def hollow_box_inertia(mass: float, a: float, b: float, c: float, t: float) -> Tuple[float, float, float]:
    """(Ix, Iy, Iz) for a thick-walled hollow cuboid shell of uniform wall
    thickness `t`, outer dims a(x) * b(y) * c(z), and total shell mass
    `mass` -- computed as the outer solid box's inertia minus the inner
    cavity's, where the cavity is treated as a same-density solid block of
    mass m = mass * (a-2t)(b-2t)(c-2t)/(abc) so it subtracts out cleanly."""
    inner_a, inner_b, inner_c = a - 2.0 * t, b - 2.0 * t, c - 2.0 * t
    if inner_a <= 0 or inner_b <= 0 or inner_c <= 0:
        raise ValueError(
            f"trunk shell wall thickness ({t:.4f} m) is too large for the trunk's "
            f"outer dims ({a:.4f} x {b:.4f} x {c:.4f} m) -- the inner cavity would "
            "have zero or negative extent on at least one axis. Reduce "
            "TRUNK_SHELL_THICKNESS or increase the trunk dims."
        )
    cavity_mass = mass * (inner_a * inner_b * inner_c) / (a * b * c)
    ix = _solid_box_axis_inertia(mass, b, c) - _solid_box_axis_inertia(cavity_mass, inner_b, inner_c)
    iy = _solid_box_axis_inertia(mass, a, c) - _solid_box_axis_inertia(cavity_mass, inner_a, inner_c)
    iz = _solid_box_axis_inertia(mass, a, b) - _solid_box_axis_inertia(cavity_mass, inner_a, inner_b)
    return ix, iy, iz


def _fmt(x: float, p: int = 6) -> str:
    return f"{float(x):.{p}f}"


def _vec(*xs) -> str:
    return " ".join(_fmt(x) for x in xs)


def _vec3(v) -> str:
    return " ".join(_fmt(x) for x in v)


# ---------------------------------------------------------------------------
# Geometry rules
# ---------------------------------------------------------------------------
def check_geometry(spec, act_abad, act_hip) -> None:
    r_abad, _ = _housing_dims(act_abad)
    yyy = spec.trunk_width / 2.0 - r_abad + spec.clearance_trunk2abadAct_y
    if yyy <= 0:
        raise ValueError(
            f"abad actuator radius ({r_abad:.4f} m) doesn't fit within half the "
            f"trunk width ({spec.trunk_width / 2.0:.4f} m) -- increase trunk_width "
            "or pick a smaller act_abduction."
        )
    r_hip, _ = _housing_dims(act_hip)
    under_sqrt = (yyy + spec.clearance_abadAct2hipAct_y) ** 2 - r_hip ** 2
    if under_sqrt < 0:
        raise ValueError(
            f"hip actuator radius ({r_hip:.4f} m) is too large for the abad "
            f"housing's y-offset ({yyy:.4f} m) + clearance_abadAct2hipAct_y "
            f"({spec.clearance_abadAct2hipAct_y:.4f} m) -- the hip-offset formula's sqrt "
            "term would be negative. Increase trunk_width, reduce act_hip's "
            "radius, or increase clearance_abadAct2hipAct_y."
        )


# ---------------------------------------------------------------------------
# Actuator housing layout: chains abad -> hip -> knee clearance-based offset
# formulas into base_Link-relative positions.
# ---------------------------------------------------------------------------
def _cluster_layout(side: str, spec, act_abad, act_hip, act_knee) -> Dict[str, np.ndarray]:
    sign = SIDE_SIGN[side]
    r_abad, hl_abad = _housing_dims(act_abad)
    r_hip, hl_hip = _housing_dims(act_hip)
    r_knee, hl_knee = _housing_dims(act_knee)

    x_offset_trunk2abadAct = hl_abad + spec.clearance_trunk2abadAct_x
    y_offset_trunk2abadAct = spec.trunk_width / 2.0 - r_abad + spec.clearance_trunk2abadAct_y
    z_offset_trunk2abadAct = -(spec.trunk_thickness / 2.0 + r_abad + spec.clearance_trunk2abadAct_z)
    abad_pos = np.array([x_offset_trunk2abadAct, sign * y_offset_trunk2abadAct, z_offset_trunk2abadAct])

    x_offset_abadAct2hipAct = -(hl_abad + r_hip + spec.clearance_abadAct2hipAct_x)
    under_sqrt = max((y_offset_trunk2abadAct + spec.clearance_abadAct2hipAct_y) ** 2 - r_hip ** 2, 0.0)
    y_offset_abadAct2hipAct = -sign * (math.sqrt(under_sqrt) - 2.0 * hl_hip)   # NOTE: inverted sign vs abad/knee -- established explicitly when this formula was first given (hip's y-offset is "negative for left, positive for right", the opposite of abad/knee's convention)
    z_offset_abadAct2hipAct = spec.clearance_abadAct2hipAct_z
    hip_pos_rel = np.array([x_offset_abadAct2hipAct, y_offset_abadAct2hipAct, z_offset_abadAct2hipAct])

    x_offset_hipAct2kneeAct = spec.clearance_hipAct2kneeAct_x
    y_offset_hipAct2kneeAct = sign * (hl_hip + (2.0 * THIGH_BOX_HALF[1] + spec.clearance_hipAct2kneeAct_y) + hl_knee)
    z_offset_hipAct2kneeAct = spec.clearance_hipAct2kneeAct_z
    knee_pos_rel = np.array([x_offset_hipAct2kneeAct, y_offset_hipAct2kneeAct, z_offset_hipAct2kneeAct])

    # thigh_link sits centered in the gap between the hip and knee housing faces
    x_offset_kneeAct2thigh = spec.clearance_kneeAct2thigh_x
    y_offset_kneeAct2thigh = -sign * (hl_knee + THIGH_BOX_HALF[1] + spec.clearance_kneeAct2thigh_y)
    z_offset_kneeAct2thigh = spec.clearance_kneeAct2thigh_z
    thigh_pos_rel = np.array([x_offset_kneeAct2thigh, y_offset_kneeAct2thigh, z_offset_kneeAct2thigh])

    return {
        "abad_pos": abad_pos, "hip_pos_rel": hip_pos_rel, "knee_pos_rel": knee_pos_rel,
        "thigh_pos_rel": thigh_pos_rel,
        "r_abad": r_abad, "hl_abad": hl_abad, "r_hip": r_hip, "hl_hip": hl_hip,
        "r_knee": r_knee, "hl_knee": hl_knee,
    }


def _act_geom_block(r: float, hl: float, quat: str, rgba: str) -> str:
    return (f'<geom class="visual" type="cylinder" size="{_vec(r, hl)}" quat="{quat}" rgba="{rgba}"/>\n'
            f'          <geom class="collision" type="cylinder" size="{_vec(r, hl)}" quat="{quat}"/>')


def _act_inertial_block(mass: float, r: float, hl: float, quat: str) -> str:
    i_perp, i_axial = cyl_inertia(mass, r, 2.0 * hl)
    return (f'<inertial pos="0 0 0" quat="{quat}" mass="{_fmt(mass)}" '
            f'diaginertia="{_vec(i_perp, i_perp, i_axial)}"/>')


def _leg_xml(side: str, spec, act_abad, act_hip, act_knee, act_ankle) -> str:
    L = _cluster_layout(side, spec, act_abad, act_hip, act_knee)
    abad_axis = "-1 0 0" if side == "R" else "1 0 0"

    # thigh: a single cuboid, pointing along THIGH_BOX_DIR. Its LENGTH is
    # LINK_BOX_LENGTH_FRAC of the full hip-to-knee span, but it's CENTERED on
    # the full span's midpoint -- so it stops short of both the hip and knee
    # joints by equal margins, rather than starting flush at the hip and
    # only leaving a gap at the knee end.
    thigh_available = spec.thigh_length
    if thigh_available <= 0:
        raise ValueError(
            f"thigh_length ({spec.thigh_length:.4f} m) is too short to clear "
            f"the knee actuator's own radius ({L['r_knee']:.4f} m) -- the thigh "
            "box would have zero or negative length. Increase thigh_length."
        )
    box_half_len = (LINK_BOX_LENGTH_FRAC * thigh_available) / 2.0
    box_center_local = (thigh_available / 2.0) * THIGH_BOX_DIR
    box_quat = _vec3(THIGH_BOX_QUAT)
    box_size = _vec(THIGH_BOX_HALF[0], THIGH_BOX_HALF[1], box_half_len)

    shank_pos = spec.thigh_length * THIGH_DIR_XZ   # (x, z) in the sagittal plane
    shank_pos_rel = f"{_fmt(shank_pos[0])} 0.0 {_fmt(shank_pos[1])}"

    # shank box: same LINK_BOX_LENGTH_FRAC convention -- hangs straight down
    # (-z) from shank_{side}_Link's own origin, but CENTERED on the full
    # knee-to-ankle span's midpoint (half of spec.shank_length), so it stops
    # short of both the knee and ankle joints by equal margins.
    shank_half_len = (LINK_BOX_LENGTH_FRAC * spec.shank_length) / 2.0
    shank_box_size = _vec(SHANK_BOX_HALF[0], SHANK_BOX_HALF[1], shank_half_len)
    shank_box_pos = f"0.0 0.0 {_fmt(-spec.shank_length / 2.0)}"

    # ankle_{side}_act sits shank_length straight down (-z) from
    # shank_{side}_Link's own origin, plus a small clearance offset.
    ankle_pos = np.array([
        spec.clearance_shank2ankleAct_x,
        spec.clearance_shank2ankleAct_y,
        -spec.shank_length + spec.clearance_shank2ankleAct_z,
    ])
    ankle_pos_rel = _vec3(ankle_pos)

    foot_pos_rel = _vec(0 ,0, 0)

    r_ankle, hl_ankle = _housing_dims(act_ankle)
    x_offset_ankleAct2foot = spec.clearance_ankleAct2foot_x
    z_offset_ankleAct2foot = -(r_ankle + FOOT_BOX_HALF[1] + spec.clearance_ankleAct2foot_z)
    z_bottom_ankleAct2foot = -(r_ankle + 2.0 * FOOT_BOX_HALF[1] + spec.clearance_ankleAct2foot_z)
    foot_size = _vec(spec.foot_length / 2.0, FOOT_BOX_HALF[0], FOOT_BOX_HALF[1])

    knee_lo, knee_hi = KNEE_RANGE

    return f"""
      <!-- ==================== {'RIGHT' if side == 'R' else 'LEFT'} LEG (actuator cluster + thigh) ==================== -->
      <body name="abad_{side}_act" pos="{_vec3(L['abad_pos'])}">
        <!-- welded to base: no <joint> element -->
        {_act_inertial_block(act_abad.mass, L['r_abad'], L['hl_abad'], QUAT_ALIGN_X)}
        {_act_geom_block(L['r_abad'], L['hl_abad'], QUAT_ALIGN_X, LINK_COLORS['abad'])}

        <body name="hip_{side}_act" pos="{_vec3(L['hip_pos_rel'])}">
          <joint class="abad" name="abad_{side}_Joint" axis="{abad_axis}" range="{_vec(*ABAD_RANGE)}"/>
          {_act_inertial_block(act_hip.mass, L['r_hip'], L['hl_hip'], QUAT_ALIGN_Y)}
          {_act_geom_block(L['r_hip'], L['hl_hip'], QUAT_ALIGN_Y, LINK_COLORS['hip'])}

          <body name="knee_{side}_act" pos="{_vec3(L['knee_pos_rel'])}">
            <joint class="hip" name="hip_{side}_Joint" axis="0 -1 0" range="{_vec(*HIP_RANGE)}"/>
            {_act_inertial_block(act_knee.mass, L['r_knee'], L['hl_knee'], QUAT_ALIGN_Y)}
            {_act_geom_block(L['r_knee'], L['hl_knee'], QUAT_ALIGN_Y, LINK_COLORS['knee'])}

            <body name="thigh_{side}_Link" pos="{_vec3(L['thigh_pos_rel'])}">
              <geom class="visual" type="box" size="{box_size}" pos="{_vec3(box_center_local)}" quat="{box_quat}" rgba="{LINK_COLORS['thigh']}"/>
              <geom class="collision" type="box" size="{box_size}" pos="{_vec3(box_center_local)}" quat="{box_quat}" density="{_fmt(THIGH_DENSITY)}"/>

              <body name="shank_{side}_Link" pos="{shank_pos_rel}" quat="{SHANK_QUAT}">
                <joint class="knee" name="knee_{side}_Joint" axis="0 -1 0" range="{_fmt(knee_lo)} {_fmt(knee_hi)}"/>
                <geom class="visual" type="box" size="{shank_box_size}" pos="{shank_box_pos}" rgba="{LINK_COLORS['shank']}"/>
                <geom class="collision" type="box" size="{shank_box_size}" pos="{shank_box_pos}" density="{_fmt(SHANK_DENSITY)}"/>

                <body name="ankle_{side}_act" pos="{ankle_pos_rel}" quat="{ANKLE_QUAT}">
                  {_act_inertial_block(act_ankle.mass, r_ankle, hl_ankle, QUAT_ALIGN_Y)}
                  {_act_geom_block(r_ankle, hl_ankle, QUAT_ALIGN_Y, LINK_COLORS['ankle'])}

                  <body name="foot_{side}_Link" pos="{foot_pos_rel}">
                    <joint class="ankle" name="ankle_{side}_Joint" axis="0 -1 0" range="{_vec(*ANKLE_RANGE)}"/>
                    <geom class="visual" type="box" size="{foot_size}" pos="{_fmt(x_offset_ankleAct2foot)} 0.0 {_fmt(z_offset_ankleAct2foot)}" rgba="{LINK_COLORS['ankle']}"/>
                    <geom class="collision" name="foot_{side}_collision" type="box" size="{foot_size}" pos="{_fmt(x_offset_ankleAct2foot)} 0.0 {_fmt(z_offset_ankleAct2foot)}" density="{_fmt(FOOT_DENSITY)}"/>
                    <site name="foot_{side}" pos="{_fmt(x_offset_ankleAct2foot)} 0.0 {_fmt(z_bottom_ankleAct2foot)}" size="0.01"/>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>"""


# ---------------------------------------------------------------------------
# Top-level MJCF assembly
# ---------------------------------------------------------------------------
def build_xml(spec) -> str:
    act_abad, act_hip, act_knee, act_ankle = spec.actuators()
    check_geometry(spec, act_abad, act_hip)

    # trunk_Link is a hollow shell (encloses the battery/electronics), not a
    # solid block -- mass = shell surface area * wall thickness * material
    # density, plus TRUNK_EXTRA_MASS for whatever's inside. Inertia is the
    # SUM of two separate rigid bodies sharing the same center: (1) the
    # thick-walled hollow shell itself (outer solid box minus its inner
    # cavity, both at the shell's own mass/density), and (2) a solid cuboid
    # "battery pack" of mass TRUNK_EXTRA_MASS. The battery box is assumed to
    # fill the shell's own inner cavity dims (trunk dims minus 2x wall
    # thickness on each axis) -- there's no separate battery-size constant,
    # so this is the natural "whatever's inside just fits the cavity"
    # assumption; tune TRUNK_SHELL_THICKNESS or add a dedicated battery-size
    # constant if that's ever not a good approximation.
    a, b, c, t = spec.trunk_length, spec.trunk_width, spec.trunk_thickness, TRUNK_SHELL_THICKNESS
    trunk_surface_area = 2.0 * (a * b + b * c + c * a)
    m_shell = trunk_surface_area * t * TRUNK_SHELL_DENSITY
    m_trunk = m_shell + TRUNK_EXTRA_MASS

    shell_Ixx, shell_Iyy, shell_Izz = hollow_box_inertia(m_shell, a, b, c, t)
    inner_a, inner_b, inner_c = a - 2.0 * t, b - 2.0 * t, c - 2.0 * t
    batt_Ixx = _solid_box_axis_inertia(TRUNK_EXTRA_MASS, inner_b, inner_c)
    batt_Iyy = _solid_box_axis_inertia(TRUNK_EXTRA_MASS, inner_a, inner_c)
    batt_Izz = _solid_box_axis_inertia(TRUNK_EXTRA_MASS, inner_a, inner_b)

    trunk_Ixx = shell_Ixx + batt_Ixx
    trunk_Iyy = shell_Iyy + batt_Iyy
    trunk_Izz = shell_Izz + batt_Izz

    base_z = spec.body_height   # direct spawn height -- no hip_z_drop composition in this topology

    right_xml = _leg_xml("R", spec, act_abad, act_hip, act_knee, act_ankle)
    left_xml = _leg_xml("L", spec, act_abad, act_hip, act_knee, act_ankle)

    limits = {"abad": act_abad.peak_torque, "hip": act_hip.peak_torque,
              "knee": act_knee.peak_torque, "ankle": act_ankle.peak_torque}

    def _default_class(name: str) -> str:
        dyn = JOINT_DYNAMICS[name]
        lim = _fmt(abs(limits[name]))
        return f"""    <default class="{name}">
      <joint damping="{_fmt(dyn['damping'])}" armature="{_fmt(dyn['armature'])}" frictionloss="{_fmt(dyn['frictionloss'])}"/>
      <motor ctrlrange="-{lim} {lim}" forcerange="-{lim} {lim}"/>
    </default>"""

    defaults_xml = "\n".join(_default_class(n) for n in ("abad", "hip", "knee", "ankle"))

    # fixed small joint angles, matching design_biped.xml's own convention --
    # this topology's leg isn't a simple 2-link chain, so there's no clean
    # closed-form IK for a target standing depth.
    qpos_leg = [0.0, 0.1, 0.15, -0.1]
    qpos = [0.0, 0.0, base_z, 1.0, 0.0, 0.0, 0.0] + qpos_leg * 2
    qpos_str = " ".join(_fmt(q) for q in qpos)

    actuators_xml = "\n".join(
        f'    <motor class="{joint}" name="{joint}_{side}_motor" joint="{joint}_{side}_Joint"/>'
        for side in LEG_SIDES for joint in ("abad", "hip", "knee", "ankle")
    )

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
      <geom contype="0" conaffinity="0" group="2" rgba="0.7 0.7 0.75 1" density="0"/>
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
      <inertial pos="0 0 0" mass="{_fmt(m_trunk)}" diaginertia="{_vec(trunk_Ixx, trunk_Iyy, trunk_Izz)}"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      <geom class="visual" type="box" size="{_vec(spec.trunk_length / 2.0, spec.trunk_width / 2.0, spec.trunk_thickness / 2.0)}" rgba="0.79216 0.81961 0.93333 1"/>
      <geom class="collision" type="box" size="{_vec(spec.trunk_length / 2.0, spec.trunk_width / 2.0, spec.trunk_thickness / 2.0)}"/>
{right_xml}
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