# MJCF geometry rules (`builder.py`)

Source-of-truth constraints and fixed hardware constants `build_xml` uses
when placing bodies/geoms in the leg chain. When adding new geometry, check
it against these before hand-tuning offsets.

`builder.py` builds the ACTUATOR-CLUSTER topology (matching
`design_biped.xml`/`new_biped.xml`), not `models/tron/biped.xml`'s own
simple `abad_Link->hip_Link->knee_Link->ankle_Link` chain -- an earlier
version of this file built that instead. Each leg is:

```
base_Link -> abad_{SIDE}_act -> hip_{SIDE}_act -> knee_{SIDE}_act
    -> thigh_{SIDE}_Link -> shank_{SIDE}_Link -> ankle_{SIDE}_act
```

## Conventions

- All lengths are in metres, matching `DesignSpec` field units.
- Box `size=` in MJCF is a half-extent. `spec.trunk_length`/`trunk_width`/
  `trunk_thickness` and `spec.foot_length`/`foot_width`/`foot_height` are
  **full** dimensions (edge-to-edge), so those geoms use e.g.
  `spec.trunk_length/2` as their `size`.
- Cylinder `size="radius half_length"` is measured along the cylinder's own
  local z-axis before any `quat` rotation is applied. `abad_{SIDE}_act`'s
  cylinder is oriented along body/parent-frame **x** (`QUAT_ALIGN_X`, 90 deg
  about y) -- the ab/ad joint's own axis. Every other actuator cylinder
  (`hip_{SIDE}_act`/`knee_{SIDE}_act`/`ankle_{SIDE}_act`, plus the thigh's
  own cylinder) uses `QUAT_ALIGN_Y` (90 deg about x), swinging its axis onto
  **y** -- the hip/knee/ankle joints' shared rotation axis.
- A body chain's position is always relative to its parent body, not world.

## Rule 1 -- abad actuator must fit within the trunk

```
trunk_width / 2 > abad_actuator_radius
```

`yyy = trunk_width/2 - r_abad` is the abad housing's unsigned y-offset --
reused inside the hip-offset formula's sqrt term too (below). If the picked
abad actuator's radius is too large for the trunk, `yyy` goes negative and
nothing downstream makes sense. Enforced as a hard `ValueError` in
`check_geometry(spec, act_abad, act_hip)`.

## Rule 2 -- hip actuator radius must fit the abad-offset geometry

```
(yyy + clearance_hip_y)^2 >= hip_actuator_radius^2
```

The hip-offset formula's sqrt term (below) goes imaginary if the hip
actuator's radius is too large relative to `yyy + clearance_hip_y`.
Enforced as a hard `ValueError` in `check_geometry`.

Unlike the old chain topology, there's no "home-pose depth must be
reachable" rule here -- this topology's leg isn't a simple 2-link chain (the
actuator cluster plus a tangent-cylinder thigh box plus a fixed shank/ankle
bend don't reduce to a closed-form 2-link IK), so the home keyframe just
uses fixed small joint angles (`0, 0.1, 0.15, -0.1` per leg, matching
`design_biped.xml`'s own convention) instead of solving for a target depth.
`body_height` sets `base_Link`'s spawn Z directly, with no reachability
check against `thigh_length`/`shank_length`.

## Actuator-cluster placement formulas

`abad_{SIDE}_act`, `hip_{SIDE}_act`, `knee_{SIDE}_act` are chained -- each
positioned relative to the previous one -- by the clearance-based x/y/z
offset formulas given in chat:

```
trunk_length = input
trunk_width = input
trunk_thickness = input

clearance_abad_x = input
clearance_abad_y = input
clearance_abad_z = input

constant_thigh_width = constant


-------------------------------------------------------------------

# abad_{SIDE}_act, relative to base_Link:
x_abad = thickness_abad / 2 + clearance_trunk2abadAct_x                                   # (+ both legs)
yyy = trunk_width/2 - r_abad + clearance_trunk2abadAct_y
y_abad = +-yyy                                                 # + left, - right
z_abad = -(trunk_thickness/2 + r_abad + clearance_trunk2abadAct_z)      # (- both legs)

# hip_{SIDE}_act, relative to abad_{SIDE}_act:
x_hip = -(thickness_abad/2 + r_hip + clearance_abadAct2hipAct_x)          # (- both legs)
y_hip = -+(sqrt((yyy + clearance_abadAct2hipAct_y)^2 - r_hip^2) - thickness_hip)
        # NOTE: inverted sign vs abad/knee -- established explicitly when
        # this formula was first given (hip's y-offset is "negative for
        # left, positive for right", the opposite of abad/knee's convention)
z_hip = 0

# knee_{SIDE}_act, relative to hip_{SIDE}_act:
x_knee = 0
y_knee = +-(thickness_hip/2 + (width_thigh + clearance_hipAct2kneeAct_y) + thickness_knee/2)  # + left, - right
z_knee = 0

#knee act to thigh link:
x_thigh = 0
y_thigh = thickness_knee/2 + constant_thigh_width
z_thigh + 0

```

`thickness_X` = the full axial length of actuator X's housing cylinder
(`2 * X.length/2`, from the resolved `CatalogActuator`/`Actuator`'s `length`
field). `r_X` = `X.width / 2`. `width_thigh = 2 * ORIG_THIGH_BOX_HALF[1]`
(the thigh box's own fixed cross-section, reused as the gap-fill width
between the hip and knee housings). `clearance_abad_z`/`clearance_hip_x`/
`clearance_hip_y` are `DesignSpec` fields (50mm/20mm/10mm at baseline).

No clearance term was given for the knee-actuator offset formula, so none is
applied there.

## Thigh, shank, ankle

**`thigh_{SIDE}_Link`** (welded to `knee_{SIDE}_act`, no `<joint>`): a
cylinder (radius = knee actuator's radius, length = the full hip/knee
housing gap) plus a cuboid (`ORIG_THIGH_BOX_HALF = (0.025, 0.016, 0.125)`,
a fixed cross-section+length, NOT `spec.thigh_length` -- see below) whose
small end face sits tangent to that cylinder's side, pointing along
`THIGH_BOX_DIR` -- biped.xml's real hip->knee bone direction (X-Z plane
only, y dropped), a fixed hardware/CAD angle. No explicit `<inertial>`:
mass/COM/inertia auto-derived by MuJoCo from geom density x volume
(aluminum, 2700 kg/m^3; visual-class geoms use `density="0"` so they don't
double-count against the collision geoms).

**`shank_{SIDE}_Link`** (child of `thigh_{SIDE}_Link`): positioned at
`spec.thigh_length * THIGH_DIR_XZ` (y always 0) -- i.e. `thigh_length` scales
ALONG the fixed bend direction above; it does not change the angle. Carries
the belt-driven `knee_{SIDE}_Joint` (axis `0 -1 0`, `KNEE_RANGE` -- same
range both legs, matching biped.xml's real `knee_R_Joint`). Orientation is the
fixed `SHANK_QUAT` (biped.xml's real thigh->shank bend). Geometry/inertial
is a placeholder, reused verbatim from biped.xml's `knee_R_Link`/
`knee_L_Link` box (`SHANK_BOX_HALF`, `SHANK_MASS`, `SHANK_DIAGINERTIA`) --
not yet `DesignSpec`-controlled.

**`ankle_{SIDE}_act`** (child of `shank_{SIDE}_Link`): positioned at
`spec.shank_length * SHANK_DIR` -- same scale-along-a-fixed-direction
convention as the thigh. Combines the ankle actuator housing (cylinder,
catalog radius/length) and the foot (cuboid, sized from
`spec.foot_length`/`foot_width`/`foot_height`) in one body -- matching the
real hardware and the actuator catalog's note that the ankle motor has "no
separate body". Foot box center/site use the same `FOOT_X_OFFSET_FRAC`
(0.05) / `FOOT_ANKLE_GAP` (0.02 m) convention as the old chain topology, so
they reproduce biped.xml's real foot placement exactly at the baseline foot
dimensions. Ankle inertial (`ANKLE_ACT_MASS`/`ANKLE_ACT_DIAGINERTIA`) is
biped.xml's real combined `ankle_R_Link` value -- catalog said the ankle
motor's own mass "wasn't queried" -- so it does NOT scale with
`spec.foot_*` the way the foot geom itself does.

Because `thigh_length`/`shank_length` scale a FIXED direction rather than
being re-derived from a full 3D vector, the baseline defaults
(`0.3522218416`/`0.3435109359`, biped.xml's real full-3D bone lengths,
including a small dropped y-component for the thigh) land the shank/ankle
about 2mm further out than `design_biped.xml`'s literal hardcoded numbers --
a known, deliberate trade-off for having a tunable length at all, not a bug.

## Fixed hardware constants (not part of `DesignSpec`)

| Constant | Value | What it is |
|---|---|---|
| `THIGH_WIDTH_GAP` | 0.030 m | `width_thigh` / `constant_thigh_width` -- fixed gap-fill width used by both the knee-offset formula and the thigh-placement formula (same constant, not `DesignSpec`-controlled) |
| `ORIG_THIGH_BOX_HALF` | (0.025, 0.016, 0.125) m | Thigh box cross-section + length -- fixed, not `spec.thigh_length` |
| `THIGH_DENSITY` | 2700 kg/m^3 | Aluminum, used to auto-derive thigh mass/inertia from geometry |
| `SHANK_BOX_HALF` / `SHANK_BOX_POS` | (0.0125, 0.016, 0.15) m / (0,0,-0.15) | Placeholder shank geometry, biped.xml's real box |
| `SHANK_MASS` / `SHANK_DIAGINERTIA` | 1.0 kg / (0.04326, 0.039749, 0.003806) | Placeholder shank inertial, biped.xml's real value |
| `SHANK_QUAT` | `0.939692 0 -0.342022 0` | Fixed thigh->shank bend, biped.xml's real CAD angle |
| `KNEE_RANGE` | (-1.21357, 1.17267) | Same both legs, matching biped.xml's real `knee_R_Joint` range |
| `ANKLE_ACT_MASS` / `ANKLE_ACT_DIAGINERTIA` | 0.7 kg / (0.002619, 0.002386, 0.0003387) | biped.xml's real combined ankle-link inertial (motor mass "not queried") |
| `ANKLE_QUAT` | `0.939681 0 0.342052 0` | Fixed shank->ankle bend, biped.xml's real CAD angle |
| `ANKLE_RANGE` | (-0.791917, 1.30899) | Same both legs, from biped.xml |
| `ABAD_RANGE` / `HIP_RANGE` | (-0.38397, 1.39626) / (-1.39626, 1.01229) | Same both legs, from biped.xml |
| `FOOT_X_OFFSET_FRAC` | 0.05 | Foot box centre offset forward of the ankle joint, as a fraction of `foot_length` |
| `FOOT_ANKLE_GAP` | 0.02 m | Vertical gap from the ankle joint down to the top of the foot box |
| `HOUSING_GAP` | 0.005 m | Unused by the current placement formulas; kept for reference |
| `JOINT_DYNAMICS` | see `biped.xml`'s `<default>` | Per-class `damping`/`armature`/`frictionloss` |
| Trunk density | 899.5 kg/m^3 | Hardcoded in `build_xml`, calibrated so the baseline trunk box masses ~12 kg |

## Torque limits and actuator picks

`<motor>` (direct-torque) actuators inside `<default>` classes -- matching
`biped.xml`. Each joint class's `ctrlrange`/`forcerange` is `+-peak_torque`
of the `DesignSpec` actuator picked for that joint *type*
(`act_abduction`/`act_hip`/`act_knee`/`act_ankle`), mirrored across both
legs, independent of where that actuator's housing geometry is drawn.