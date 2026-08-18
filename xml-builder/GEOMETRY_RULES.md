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

Unlike the old chain topology, there's no "home-pose depth must be
reachable" rule here -- this topology's leg isn't a simple 2-link chain (the
actuator cluster plus a thigh box plus a fixed shank/ankle bend don't
reduce to a closed-form 2-link IK), so the home keyframe just
uses fixed small joint angles (`0, 0.1, 0.15, -0.1` per leg, matching
`design_biped.xml`'s own convention) instead of solving for a target depth.
`body_height` sets `base_Link`'s spawn Z directly, with no reachability
check against `thigh_length`/`shank_length`.

## Actuator-cluster placement formulas

`abad_{SIDE}_act`, `hip_{SIDE}_act`, `knee_{SIDE}_act`, `thigh_{SIDE}_Link`,
`ankle_{SIDE}_act`, and `foot_{SIDE}_Link` are all chained -- each positioned
relative to the previous one -- by the clearance-based x/y/z offset formulas
below. Every `clearance_*` symbol is a `DesignSpec` field (see its class
docstring in `generate.py` for the free/fixed-constant split -- fixed-constant
ones default to 0.0 unless a caller overrides them, but are still read here,
same as the free ones). `thigh_width` below is NOT a `DesignSpec` field --
it's `builder.py`'s fixed `THIGH_BOX_HALF[1]` constant (see "Thigh, shank,
ankle" below).

```
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
z_hip = clearance_abadAct2hipAct_z

# knee_{SIDE}_act, relative to hip_{SIDE}_act:
x_knee = clearance_hipAct2kneeAct_x
y_knee = +-(thickness_hip/2 + (2*THIGH_BOX_HALF[1] + clearance_hipAct2kneeAct_y) + thickness_knee/2)  # + left, - right -- 2*THIGH_BOX_HALF[1] = thigh box's FULL extent along Y
z_knee = clearance_hipAct2kneeAct_z

# thigh_{SIDE}_Link, relative to knee_{SIDE}_act:
x_thigh = clearance_kneeAct2thigh_x
y_thigh = -+(thickness_knee/2 + THIGH_BOX_HALF[1] + clearance_kneeAct2thigh_y)   # sign opposite y_knee's (tangent point sits back toward the hip side, not out past the knee) -- HALF extent here, not full
z_thigh = clearance_kneeAct2thigh_z

# ankle_{SIDE}_act, relative to shank_{SIDE}_Link -- layered on top of the
# fixed-direction shank_length*SHANK_DIR scaling, see "Thigh, shank, ankle" below:
x_ankleAct = clearance_shank2ankleAct_x
y_ankleAct = clearance_shank2ankleAct_y
z_ankleAct = clearance_shank2ankleAct_z

# foot_{SIDE}_Link, relative to ankle_{SIDE}_act:
x_foot = clearance_ankleAct2foot_x
y_foot = clearance_ankleAct2foot_y
z_foot = clearance_ankleAct2foot_z
```

`thickness_X` = the full axial length of actuator X's housing cylinder
(`2 * X.length/2`, from the resolved `CatalogActuator`/`Actuator`'s `length`
field). `r_X` = `X.width / 2`. `THIGH_BOX_HALF[1]` (0.016 m) replaces what
used to be a single decoupled "gap-fill width" value (`builder.py` constant
`THIGH_WIDTH_GAP` -> spec field `thigh_width` -> now gone). It's index `[1]`,
not `[0]`, that belongs here: the thigh box's `size=` is
`_vec(THIGH_BOX_HALF[0], THIGH_BOX_HALF[1], box_half_len)`, i.e. `[0]` is
the box's local-X half-extent (within the sagittal X-Z plane) and `[1]` is
its local-Y half-extent -- and since `THIGH_BOX_QUAT` only rotates about Y,
the box's local-Y axis stays aligned with GLOBAL Y no matter the bend
angle, the same axis `y_knee`/`y_thigh` are measured along. Since
`THIGH_BOX_HALF[1]` is a HALF-extent, not a full one, `y_knee` above uses
`2*THIGH_BOX_HALF[1]` (the box's FULL extent along Y, since that's the
clear span the box needs) while `y_thigh` uses `THIGH_BOX_HALF[1]` directly
(already a half-extent). All `clearance_*` fields default to 0.0 except
`clearance_trunk2abadAct_z`/`clearance_abadAct2hipAct_x`/`_y`/
`clearance_hipAct2kneeAct_y` (free, 50mm/20mm/10mm/10mm at baseline) and
`clearance_ankleAct2foot_z` (fixed-constant, 30mm at baseline -- must stay
positive since the foot sits below the ankle actuator).

**`z_abad` discrepancy**: the source notes for this formula say
`z_abad = -(trunk_length/2 + r_abad + clearance_trunk2abadAct_z)`, but
`builder.py` uses `trunk_thickness/2`, not `trunk_length/2`, matching the
trunk box's actual z half-extent and reproducing `design_biped.xml`'s real
geometry to 6 decimal places -- `trunk_length/2` above looks like a
copy/paste slip in the notes, not an intended change.

## Thigh, shank, ankle

**`thigh_{SIDE}_Link`** (welded to `knee_{SIDE}_act`, no `<joint>`): a
single cuboid (`THIGH_BOX_HALF = (0.025, 0.016)`, a fixed cross-section --
box LENGTH is computed separately per-spec, see below -- NOT
`spec.thigh_length`/`thigh_thickness`), pointing along `THIGH_BOX_DIR` --
biped.xml's real hip->knee bone direction (X-Z plane only, y dropped), a
fixed hardware/CAD angle. Its near face is offset from `thigh_{SIDE}_Link`'s
own origin by the knee actuator's radius (`L['r_knee']`), so it doesn't
visually overlap `knee_{SIDE}_act`'s own cylinder geometry -- there used to
be an actual tangent cylinder filling that gap, since removed in favor of
just leaving the offset. No explicit `<inertial>`: mass/COM/inertia
auto-derived by MuJoCo from geom density x volume, where the density
(`THIGH_DENSITY`) is itself derived from a linear mass model, not a fixed
material constant -- see "Fixed hardware constants" below; visual-class
geoms use `density="0"` so they don't double-count against the collision
geoms.

**`shank_{SIDE}_Link`** (child of `thigh_{SIDE}_Link`): positioned at
`spec.thigh_length * THIGH_DIR_XZ` (y always 0) -- i.e. `thigh_length` scales
ALONG the fixed bend direction above; it does not change the angle. Carries
the belt-driven `knee_{SIDE}_Joint` (axis `0 -1 0`, `KNEE_RANGE` -- same
range both legs, matching biped.xml's real `knee_R_Joint`). Orientation is the
fixed `SHANK_QUAT` (biped.xml's real thigh->shank bend). Geometry is a
placeholder cross-section (`SHANK_BOX_HALF`, reused verbatim from biped.xml's
`knee_R_Link`/`knee_L_Link` box -- NOT `spec.shank_width`/`shank_thickness`,
still not spec-controlled). No explicit `<inertial>`: mass/inertia
auto-derived by MuJoCo from geom density x volume (`SHANK_DENSITY`,
lambda-derived -- same convention as the thigh/foot, not a fixed
mass/diaginertia anymore).

**`ankle_{SIDE}_act`** (child of `shank_{SIDE}_Link`, no `<joint>` -- welded,
same treatment as abad/hip/knee's housings): positioned at
`spec.shank_length * SHANK_DIR + (clearance_shank2ankleAct_x,
clearance_shank2ankleAct_y, clearance_shank2ankleAct_z)` -- the base
position scales along the same fixed-direction convention as the thigh,
with a spec-controlled clearance offset layered on top (all default 0.0).
Geometry is just the ankle actuator housing (cylinder, catalog
radius/length); inertial comes from the catalog's `act_ankle.mass` via
`_act_inertial_block` (computed from geometry, same as abad/hip/knee), not
a fixed constant.

**`foot_{SIDE}_Link`** (child of `ankle_{SIDE}_act` -- a SEPARATE body, not
merged into the actuator housing): positioned at
`(clearance_ankleAct2foot_x, clearance_ankleAct2foot_y,
clearance_ankleAct2foot_z)` relative to `ankle_{SIDE}_act` -- all default to
0.0 except `_z`, which defaults to 0.03 m and must stay positive (the foot
sits below the ankle actuator). Carries the actual `ankle_{SIDE}_Joint` DOF
(axis `0 -1 0`, `ANKLE_RANGE`) and the `foot_{SIDE}` sensor site. The foot
box/site's own position *within* `foot_{SIDE}_Link` is a separate,
unrelated offset -- still the fixed `FOOT_X_OFFSET_FRAC` (0.05) /
`FOOT_ANKLE_GAP` (0.02 m) `builder.py` constants, so they reproduce
biped.xml's real foot placement exactly at the baseline foot dimensions.
Box/site are sized from `spec.foot_length`/`foot_width`/`foot_height`;
mass/inertia auto-derived from geom density x volume (`FOOT_DENSITY`,
lambda-derived -- same convention as the thigh).

Because `thigh_length`/`shank_length` scale a FIXED direction rather than
being re-derived from a full 3D vector, the baseline defaults
(`0.3522218416`/`0.3435109359`, biped.xml's real full-3D bone lengths,
including a small dropped y-component for the thigh) land the shank/ankle
about 2mm further out than `design_biped.xml`'s literal hardcoded numbers --
a known, deliberate trade-off for having a tunable length at all, not a bug.

## Fixed hardware constants (not part of `DesignSpec`)

| Constant | Value | What it is |
|---|---|---|
| `THIGH_BOX_HALF` | (0.025, 0.016) m | Thigh box cross-section, `(local-X half-extent, local-Y half-extent)` -- fixed, NOT `spec.thigh_thickness`; box LENGTH is computed per-spec, not stored here. `THIGH_BOX_HALF[1]` (the Y-extent -- stays aligned with global Y since `THIGH_BOX_QUAT` only rotates about Y) also doubles as the knee/thigh gap-fill width in the `y_knee`/`y_thigh` offset formulas (formerly the separate `spec.thigh_width` field); `THIGH_BOX_HALF[0]` only affects the box's extent within the sagittal X-Z plane |
| `LAMBDA_THIGH` | 4.32 kg/m | Thigh mass-per-length; `THIGH_DENSITY` is derived from this as `LAMBDA_THIGH / (width * height)` of `THIGH_BOX_HALF`, so that MuJoCo's density x volume auto-mass reduces to `M_thigh = LAMBDA_THIGH * thigh_length` regardless of box length -- see `_linear_density` in `builder.py` |
| `SHANK_BOX_HALF` | (0.0125, 0.016) m | Placeholder shank cross-section, biped.xml's real box -- NOT `spec.shank_width`/`shank_thickness` |
| `LAMBDA_SHANK` | 2.16 kg/m | Shank mass-per-length, same `_linear_density` derivation as `LAMBDA_THIGH` -- `SHANK_DENSITY` is derived from this and `SHANK_BOX_HALF` |
| `FOOT_BOX_HALF` | (0.035, 0.015) m | Foot box cross-section (half-width, half-height) -- fixed, NOT `spec.foot_width`/`foot_height` (removed fields); only `spec.foot_length` (the box's x-extent) stays spec-controlled |
| `SHANK_QUAT` | `0.939692 0 -0.342022 0` | Fixed thigh->shank bend, biped.xml's real CAD angle |
| `KNEE_RANGE` | (-1.21357, 1.17267) | Same both legs, matching biped.xml's real `knee_R_Joint` range |
| `ANKLE_QUAT` | `0.939681 0 0.342052 0` | Fixed shank->ankle bend, biped.xml's real CAD angle |
| `ANKLE_RANGE` | (-0.791917, 1.30899) | Same both legs, from biped.xml |
| `ABAD_RANGE` / `HIP_RANGE` | (-0.38397, 1.39626) / (-1.39626, 1.01229) | Same both legs, from biped.xml |
| `FOOT_X_OFFSET_FRAC` | 0.05 | Foot box/site centre offset forward of the ankle joint, as a fraction of `foot_length` -- internal to `foot_{SIDE}_Link`, unrelated to `clearance_ankleAct2foot_*` |
| `FOOT_ANKLE_GAP` | 0.02 m | Vertical gap from the ankle joint down to the top of the foot box -- same internal-to-`foot_{SIDE}_Link` caveat |
| `LAMBDA_FOOT` | 2.1 kg/m | Foot mass-per-length, same `_linear_density` derivation as `LAMBDA_THIGH` -- `FOOT_DENSITY` is derived from this and `FOOT_BOX_HALF` |
| `HOUSING_GAP` | 0.005 m | Unused by the current placement formulas; kept for reference |
| `JOINT_DYNAMICS` | see `biped.xml`'s `<default>` | Per-class `damping`/`armature`/`frictionloss` |
| `TRUNK_SHELL_THICKNESS` | 0.003 m | Trunk shell wall thickness -- `base_Link` is modeled as a hollow shell (battery/electronics inside), not a solid block; mass = surface area x thickness x `TRUNK_SHELL_DENSITY`, see `build_xml` |
| `TRUNK_SHELL_DENSITY` | 2700 kg/m^3 | Trunk shell material density (aluminum placeholder) |
| `TRUNK_EXTRA_MASS` | 0.0 kg | Fixed payload mass added on top of the shell mass (battery + electronics) -- user-supplied, tune to your actual hardware |

Ankle-actuator inertial is NOT a fixed constant here -- it comes from the
catalog's `act_ankle.mass` plus geometry-derived inertia, the same
`_act_inertial_block` treatment as abad/hip/knee.

## Torque limits and actuator picks

`<motor>` (direct-torque) actuators inside `<default>` classes -- matching
`biped.xml`. Each joint class's `ctrlrange`/`forcerange` is `+-peak_torque`
of the `DesignSpec` actuator picked for that joint *type*
(`act_abduction`/`act_hip`/`act_knee`/`act_ankle`), mirrored across both
legs, independent of where that actuator's housing geometry is drawn.