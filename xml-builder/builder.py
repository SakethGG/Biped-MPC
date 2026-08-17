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
- ``thigh_{SIDE}_Link`` is welded to ``knee_{SIDE}_act``: a cylinder
  (radius = knee actuator's radius, length = the full hip/knee-actuator gap)
  tangent to a cuboid (fixed cross-section, ``ORIG_THIGH_BOX_HALF``) that
  points along biped.xml's real hip->knee bone direction, scaled to
  ``spec.thigh_length``.
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

Still NOT spec-controlled: the thigh/shank box cross-sections
(``ORIG_THIGH_BOX_HALF``/``SHANK_BOX_HALF`` stay fixed constants below, even
though ``DesignSpec.thigh_thickness``/``shank_width``/``shank_thickness``
exist as fields -- no placement formula was given for those, only for
``thigh_width``, which *is* wired in as the knee/thigh gap-fill width) and
the foot box's own internal shape (``FOOT_X_OFFSET_FRAC``/``FOOT_ANKLE_GAP``).
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np

# ---------------------------------------------------------------------------
LEG_SIDES = ("R", "L")
SIDE_SIGN = {"R": -1.0, "L": 1.0}

# Gap kept between the ankle housing and the distal end of the body it sits
# on (no chat formula was given for ankle placement -- kept as a fixed
# hardware constant, like the rest of this file's non-spec-controlled bits).
HOUSING_GAP = 0.005

# thigh/shank box LENGTH now stays inside a fraction of the full available
# span (tangent point -> next joint, or joint -> next joint) rather than
# reaching it exactly, so consecutive link boxes don't touch/overlap at
# their shared joint -- e.g. shank_length=10 -> box length=8. Cross-sections
# stay fixed either way.
LINK_BOX_LENGTH_FRAC = 0.8

# thigh box: fixed CROSS-SECTION only (biped.xml's original thigh box) --
# its length is computed per-spec in _leg_xml (LINK_BOX_LENGTH_FRAC *
# (thigh_length - r_knee)), not this fixed half-length.
ORIG_THIGH_BOX_HALF = (0.025, 0.016, 0.125)
THIGH_DENSITY = 2700.0   # aluminum, kg/m^3 -- auto-derives thigh mass/inertia from geometry

# GEOMETRY_RULES.md's "width_thigh" / "constant_thigh_width" is now
# spec.thigh_width -- the same value used both as the knee-offset formula's
# gap-fill width and as the thigh_{side}_Link placement offset. Distinct
# from ORIG_THIGH_BOX_HALF (which sizes the thigh box's own visual/collision
# geometry, not yet spec-controlled).

# shank: fixed CROSS-SECTION + inertial only, reused verbatim from biped.xml's
# knee_R_Link/knee_L_Link (mass/inertia not yet spec-controlled -- doesn't
# rescale with the box's now-variable length, same known limitation as the
# ankle inertial below). Length is computed per-spec in _leg_xml
# (LINK_BOX_LENGTH_FRAC * shank_length), not SHANK_BOX_HALF[2] below.
SHANK_BOX_HALF = (0.0125, 0.016)   # (half-width, half-thickness) -- cross-section only
SHANK_MASS = 1.0
SHANK_DIAGINERTIA = (0.043260, 0.039749, 0.003806)
SHANK_QUAT = "0.939692 0 -0.342022 0"          # biped.xml's real thigh->shank bend, fixed
KNEE_RANGE = (-1.21357, 1.17267)   # same both legs, matching biped.xml's real knee_R_Joint range

# ankle actuator housing: now its own body (welded to shank_{side}_Link),
# same treatment as abad/hip/knee -- mass/inertia comes from the catalog via
# _act_inertial_block(act_ankle.mass, ...), not a fixed constant.
ANKLE_QUAT = "0.939681 0 0.342052 0"           # biped.xml's real shank->ankle bend, fixed
ANKLE_RANGE = (-0.791917, 1.30899)             # same both legs, from biped.xml
FOOT_X_OFFSET_FRAC = 0.05   # foot box center, fwd of the ankle joint, as a fraction of foot_length
FOOT_ANKLE_GAP = 0.02       # [m] vertical gap from the ankle joint down to the top of the foot box
FOOT_DENSITY = 1000.0       # [kg/m^3] MuJoCo's own default -- foot_{side}_Link's mass/inertia is
                            # auto-derived from geom density x volume (no explicit <inertial>),
                            # same convention as thigh/shank's THIGH_DENSITY.

ABAD_RANGE = (-0.38397, 1.39626)   # same both legs, from biped.xml
HIP_RANGE = (-1.39626, 1.01229)    # same both legs, from biped.xml

JOINT_DYNAMICS: Dict[str, Dict[str, float]] = {
    "abad":  {"damping": 1.0, "armature": 0.05, "frictionloss": 0.10},
    "hip":   {"damping": 1.0, "armature": 0.05, "frictionloss": 0.10},
    "knee":  {"damping": 1.0, "armature": 0.05, "frictionloss": 0.10},
    "ankle": {"damping": 0.5, "armature": 0.02, "frictionloss": 0.01},
}

# biped.xml's real hip->knee and knee->ankle bone directions (unit vectors) --
# fixed hardware/CAD bend angles; spec.thigh_length/shank_length scale ALONG
# these, they don't change the direction.
_THIGH_DIR_RAW_XZ = np.array([-0.200752, -0.286703])   # dropping the small
# original y-component (-0.0395), same simplification the box's own quat used
THIGH_DIR_XZ = _THIGH_DIR_RAW_XZ / np.linalg.norm(_THIGH_DIR_RAW_XZ)

_SHANK_DIR_RAW = np.array([0.0 , 0.0, -1])
SHANK_DIR = _SHANK_DIR_RAW / np.linalg.norm(_SHANK_DIR_RAW)

QUAT_ALIGN_X = "0.707107 0 0.707107 0"   # abad actuator cylinder (axis -> x)
QUAT_ALIGN_Y = "0.707107 0.707107 0 0"   # hip/knee/ankle actuator cylinder (axis -> y)

# One color per body TYPE (same for both legs), applied to visual geoms only,
# so every link is visually distinguishable in the viewer. Collision geoms
# keep the shared translucent debug rgba from the "collision" default class.
LINK_COLORS: Dict[str, str] = {
    "abad":  "0.85 0.35 0.10 1",   # orange
    "hip":   "0.15 0.45 0.85 1",   # blue
    "knee":  "0.15 0.75 0.30 1",   # green
    "thigh": "0.55 0.35 0.85 1",   # purple
    "shank": "0.10 0.65 0.65 1",   # teal
    "ankle": "0.90 0.75 0.10 1",   # gold
}


def quat_about_y_from_xz_dir(vx: float, vz: float) -> np.ndarray:
    """Quaternion for a pure Y-axis rotation mapping local z-axis (0,0,1) to
    the (normalized) direction (vx, 0, vz)."""
    theta = np.arctan2(vx, vz)
    return np.array([np.cos(theta / 2), 0.0, np.sin(theta / 2), 0.0])


THIGH_BOX_DIR = np.array([THIGH_DIR_XZ[0], 0.0, THIGH_DIR_XZ[1]])
THIGH_BOX_QUAT = quat_about_y_from_xz_dir(THIGH_DIR_XZ[0], THIGH_DIR_XZ[1])


# ---------------------------------------------------------------------------
def _housing_dims(act) -> Tuple[float, float]:
    """(radius, halflen) for an actuator's housing cylinder, from its
    catalog ``width`` (outer diameter) and ``length`` (axial thickness)."""
    return act.width / 2.0, act.length / 2.0


def cyl_inertia(mass: float, r: float, length: float) -> Tuple[float, float]:
    """(I_perp, I_axial) for a solid cylinder about its own centroid, in the
    LOCAL frame where the cylinder's symmetry axis is local z."""
    i_axial = 0.5 * mass * r * r
    i_perp = (1.0 / 12.0) * mass * (3 * r * r + length * length)
    return i_perp, i_axial


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

    x_abad = hl_abad + spec.clearance_trunk2abadAct_x
    yyy = spec.trunk_width / 2.0 - r_abad + spec.clearance_trunk2abadAct_y
    z_abad = -(spec.trunk_thickness / 2.0 + r_abad + spec.clearance_trunk2abadAct_z)
    abad_pos = np.array([x_abad, sign * yyy, z_abad])

    x_hip = -(hl_abad + r_hip + spec.clearance_abadAct2hipAct_x)
    under_sqrt = max((yyy + spec.clearance_abadAct2hipAct_y) ** 2 - r_hip ** 2, 0.0)
    y_hip = -sign * (math.sqrt(under_sqrt) - 2.0 * hl_hip)   # NOTE: inverted sign vs abad/knee -- established explicitly when this formula was first given (hip's y-offset is "negative for left, positive for right", the opposite of abad/knee's convention)
    z_hip = spec.clearance_abadAct2hipAct_z
    hip_pos_rel = np.array([x_hip, y_hip, z_hip])

    x_knee = spec.clearance_hipAct2kneeAct_x
    y_knee = sign * (hl_hip + (spec.thigh_width + spec.clearance_hipAct2kneeAct_y) + hl_knee)
    z_knee = spec.clearance_hipAct2kneeAct_z
    knee_pos_rel = np.array([x_knee, y_knee, z_knee])

    # thigh_link sits centered in the gap between the hip and knee housing faces
    x_thigh = spec.clearance_kneeAct2thigh_x
    # y_thigh = -sign * (hl_knee + spec.thigh_width + spec.clearance_kneeAct2thigh_y)
    y_thigh = -sign * (hl_knee + spec.clearance_kneeAct2thigh_y)
    z_thigh = spec.clearance_kneeAct2thigh_z
    thigh_pos_rel = np.array([x_thigh, y_thigh, z_thigh])

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

    # thigh: cylinder (knee-actuator radius, spans the full hip/knee gap) +
    # cuboid tangent to it, pointing along THIGH_BOX_DIR. Its LENGTH is
    # LINK_BOX_LENGTH_FRAC of the tangent-point -> shank-attachment span, so
    # it stops short of the shank joint by design (no visual contact) rather
    # than reaching it exactly.
    cyl_half_len = spec.thigh_width / 2.0
    thigh_available = spec.thigh_length - L["r_knee"]
    if thigh_available <= 0:
        raise ValueError(
            f"thigh_length ({spec.thigh_length:.4f} m) is too short to clear "
            f"the knee actuator's own radius ({L['r_knee']:.4f} m) -- the thigh "
            "box would have zero or negative length. Increase thigh_length."
        )
    box_half_len = (LINK_BOX_LENGTH_FRAC * thigh_available) / 2.0
    box_center_local = (L["r_knee"] + box_half_len) * THIGH_BOX_DIR
    box_quat = _vec3(THIGH_BOX_QUAT)
    box_size = _vec(ORIG_THIGH_BOX_HALF[0], ORIG_THIGH_BOX_HALF[1], box_half_len)

    shank_pos = spec.thigh_length * THIGH_DIR_XZ   # (x, z) in the sagittal plane
    shank_pos_rel = f"{_fmt(shank_pos[0])} 0.0 {_fmt(shank_pos[1])}"

    # shank box: same LINK_BOX_LENGTH_FRAC convention -- hangs straight down
    # from shank_{side}_Link's own origin (its local -z axis is very close to
    # SHANK_DIR, within ~3 deg, so this stays visually consistent with where
    # the ankle actually attaches), stopping short of the ankle joint.
    shank_half_len = (LINK_BOX_LENGTH_FRAC * spec.shank_length) / 2.0
    shank_box_size = _vec(SHANK_BOX_HALF[0], SHANK_BOX_HALF[1], shank_half_len)
    shank_box_pos = f"0.0 0.0 {_fmt(-shank_half_len)}"

    ankle_pos = spec.shank_length * SHANK_DIR + np.array([
        spec.clearance_shank2ankleAct_x,
        spec.clearance_shank2ankleAct_y,
        spec.clearance_shank2ankleAct_z,
    ])
    ankle_pos_rel = _vec3(ankle_pos)

    foot_pos_rel = _vec(spec.clearance_ankleAct2foot_x, spec.clearance_ankleAct2foot_y,
                         - spec.clearance_ankleAct2foot_z)

    r_ankle, hl_ankle = _housing_dims(act_ankle)
    x_off = FOOT_X_OFFSET_FRAC * spec.foot_length
    z_center = -(FOOT_ANKLE_GAP + spec.foot_height / 2.0)
    z_bottom = -(FOOT_ANKLE_GAP + spec.foot_height)
    foot_size = _vec(spec.foot_length / 2.0, spec.foot_width / 2.0, spec.foot_height / 2.0)

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
              <geom class="visual" type="cylinder" size="{_vec(L['r_knee'], cyl_half_len)}" quat="{QUAT_ALIGN_Y}" rgba="{LINK_COLORS['thigh']}"/>
              <geom class="collision" type="cylinder" size="{_vec(L['r_knee'], cyl_half_len)}" quat="{QUAT_ALIGN_Y}" density="{THIGH_DENSITY}"/>
              <geom class="visual" type="box" size="{box_size}" pos="{_vec3(box_center_local)}" quat="{box_quat}" rgba="{LINK_COLORS['thigh']}"/>
              <geom class="collision" type="box" size="{box_size}" pos="{_vec3(box_center_local)}" quat="{box_quat}" density="{THIGH_DENSITY}"/>

              <body name="shank_{side}_Link" pos="{shank_pos_rel}" quat="{SHANK_QUAT}">
                <joint class="knee" name="knee_{side}_Joint" axis="0 -1 0" range="{_fmt(knee_lo)} {_fmt(knee_hi)}"/>
                <inertial pos="-0.160778 0.00788018 -0.0843318" quat="0.391642 0.609187 0.580027 0.37293"
                          mass="{SHANK_MASS}" diaginertia="{_vec(*SHANK_DIAGINERTIA)}"/>
                <geom class="visual" type="box" size="{shank_box_size}" pos="{shank_box_pos}" rgba="{LINK_COLORS['shank']}"/>
                <geom class="collision" type="box" size="{shank_box_size}" pos="{shank_box_pos}"/>

                <body name="ankle_{side}_act" pos="{ankle_pos_rel}" quat="{ANKLE_QUAT}">
                  {_act_inertial_block(act_ankle.mass, r_ankle, hl_ankle, QUAT_ALIGN_Y)}
                  {_act_geom_block(r_ankle, hl_ankle, QUAT_ALIGN_Y, LINK_COLORS['ankle'])}

                  <body name="foot_{side}_Link" pos="{foot_pos_rel}">
                    <joint class="ankle" name="ankle_{side}_Joint" axis="0 -1 0" range="{_vec(*ANKLE_RANGE)}"/>
                    <geom class="visual" type="box" size="{foot_size}" pos="{_fmt(x_off)} 0.0 {_fmt(z_center)}" rgba="{LINK_COLORS['ankle']}"/>
                    <geom class="collision" name="foot_{side}_collision" type="box" size="{foot_size}" pos="{_fmt(x_off)} 0.0 {_fmt(z_center)}" density="{FOOT_DENSITY}"/>
                    <site name="foot_{side}" pos="{_fmt(x_off)} 0.0 {_fmt(z_bottom)}" size="0.01"/>
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

    m_trunk_box = 899.5 * spec.trunk_length * spec.trunk_width * spec.trunk_thickness
    trunk_Ixx = m_trunk_box * (spec.trunk_width ** 2 + spec.trunk_thickness ** 2) / 12.0
    trunk_Iyy = m_trunk_box * (spec.trunk_length ** 2 + spec.trunk_thickness ** 2) / 12.0
    trunk_Izz = m_trunk_box * (spec.trunk_length ** 2 + spec.trunk_width ** 2) / 12.0

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
      <inertial pos="0 0 0" mass="{_fmt(m_trunk_box)}" diaginertia="{_vec(trunk_Ixx, trunk_Iyy, trunk_Izz)}"/>
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