# MJCF geometry rules (`builder.py`)

Source-of-truth constraints and fixed hardware constants `build_xml` uses
when placing bodies/geoms in the leg chain. When adding new geometry, check
it against these before hand-tuning offsets. This is the biped analogue of
quadruped_xml_builder's `GEOMETRY_RULES.md` -- same idea (a handful of
enforced/derived placement rules plus a block of fixed, non-`DesignSpec`
constants), adapted from a 4-leg/3-joint actuator-housing chain to a 2-leg/
4-joint (ab/ad, hip, knee, ankle) chain matching `models/tron/biped.xml`.

## Conventions

- All lengths are in metres, matching `DesignSpec` field units.
- Box `size=` in MJCF is a half-extent. `spec.trunk_length` / `trunk_width` /
  `trunk_thickness` and `spec.foot_length` / `foot_width` / `foot_height` are
  **full** dimensions (edge-to-edge), so those geoms use e.g.
  `spec.trunk_length/2` as their `size`.
- Cylinder `size="radius half_length"` is measured along the cylinder's own
  local z-axis before any `quat` rotation is applied. Only one housing in the
  whole model is oriented along body/parent-frame **x** (`quat="1 0 1 0"`,
  90 deg about y) -- the ab/ad actuator's own housing decal on `base_Link`,
  matching the ab/ad joint's own axis. Every other actuator housing uses
  `quat="1 1 0 0"` (90 deg about x), swinging its axis onto **y** -- the
  hip/knee/ankle joints' shared rotation axis, which every proximally-mounted
  actuator drives regardless of which body its own housing sits on (verified
  by rotating each housing's local +Z through its actual `quat` in
  `biped.xml` -- see "Actuator placement and orientation" below). So a
  housing's *own* diagonal inertia `(I_axial, I_radial, I_radial)` gets
  reordered to `(I_axial, I_radial, I_radial)` for the one x-axis housing and
  `(I_radial, I_axial, I_radial)` for every y-axis housing -- see
  `_abad_own_housing_geom` / `_abad_body` / `_hip_body` / `_knee_body` in
  `builder.py`.
- A body chain's position is always relative to its parent body, not world.
- Every leg link hangs straight down its parent's local **-z** axis (like
  quadruped_xml_builder's `FR_thigh`/`FR_calf` capsules) -- there is no
  CAD-style pre-bend `quat` on any leg body, unlike the hand-tuned
  `biped.xml`. This is the main simplification vs. the real hardware file;
  see the top of `builder.py` for why.

## Rule 1 -- ab/ad joint half-spacing

```
hip_y >= ABAD_HOUSING_RADIUS + 0.01
```

The ab/ad actuator housing's centre sits `hip_y` off the trunk's x-axis
(mirrored by side). If `hip_y` is too small, the left and right housings (or
their mount brackets) would overlap across the trunk centreline.

- Enforced as a hard `ValueError` in `check_geometry` (constant `MIN_HIP_Y =
  ABAD_HOUSING_RADIUS + 0.01`). A design whose `hip_y` is below this floor
  cannot be built into an MJCF.
- Unlike quadruped_xml_builder's Rule 1/3 (which *derive* `hip_x`/`hip_y`
  from trunk dimensions and ignore the `DesignSpec` fields), this builder
  keeps `hip_x`/`hip_y`/`hip_z_drop` as free, directly-used fields -- the
  ab/ad mount offset on a biped pelvis is a more independent design choice
  than "flush against the trunk's end face," so there's no derivation rule
  here, only a minimum-clearance guard.

## Rule 2 -- home-pose depth must be reachable

```
|thigh_length - shank_length| < body_height < thigh_length + shank_length
```

`body_height` is the hip-to-ankle depth the home-pose 2-link IK
(`home_leg_angles`) solves for. Outside this band there is no real solution
(the leg can't reach that deep, or can't reach that shallow without folding
past straight).

- `check_geometry` prints a warning (not a hard error) if the raw
  `spec.body_height` falls outside the band -- `home_leg_angles` then clips
  it into `[|L1-L2|+1e-3, L1+L2-1e-3]` before solving, identical to
  quadruped_xml_builder's `home_leg_angles` clipping behaviour.

## Rule 3 -- proximal actuator mounting

```
ab/ad actuator housing + mass -> decal on base_Link (no separate body)
hip actuator housing + mass   -> lives on abad_{SIDE}_Link
knee actuator housing + mass  -> lives on hip_{SIDE}_Link
ankle actuator housing + mass -> lives on knee_{SIDE}_Link
ankle_{SIDE}_Link             -> no actuator at all, just foot structure
```

`biped.xml` mounts *every* actuator one body proximal to the joint it
drives, not co-located with its own joint. This was verified, not guessed:
rotating each housing cylinder's local +Z axis through its actual `quat` in
`biped.xml` (`Rᵀ · quat · [0,0,1]`) shows every actuator-style cylinder maps
to world/parent **-Y** (the hip/knee/ankle rotation axis, preserved through
the whole chain since every inter-body pre-bend `quat` in `biped.xml` is a
pure Y rotation) *except one*: the lone unjointed bracket geom sitting
directly in `base_Link`, just before `abad_{SIDE}_Link`, which maps to
**+X** (the ab/ad axis). That geom has no body/joint of its own, so its
mass can't live anywhere but `base_Link` -- there's nothing further up the
chain to proximally mount it on.

### Actuator placement and orientation

| Actuator | Housing lives on | Housing quat / axis | Positioned at |
|---|---|---|---|
| `act_abad` (ab/ad's own) | `base_Link` (decal, no body, no mass of its own -- folded into `base_Link`'s inertial) | `1 0 1 0` -> x | `(hip_x + ABAD_OWN_HOUSING_HALFLEN, ±hip_y, -hip_z_drop)`, matching `biped.xml`'s bracket exactly |
| `act_hip` | `abad_{SIDE}_Link` | `1 1 0 0` -> y | `ABAD_TO_HIP_OFFSET` -- the (only) attachment point of `abad_{SIDE}_Link`, right where `hip_{SIDE}_Link` attaches |
| `act_knee` | `hip_{SIDE}_Link` | `1 1 0 0` -> y | at `hip_{SIDE}_Link`'s own *proximal* origin, offset **along Y** (not Z) by `KNEE_AT_HIP_Y_OFFSET`, clustered with the ab/ad decal and hip actuator at the top of the leg -- **not** out near the knee joint |
| `act_ankle` | `knee_{SIDE}_Link` | `1 1 0 0` -> y | near `knee_{SIDE}_Link`'s *distal* end (`z = -shank_length + ANKLE_AT_KNEE_HOUSING_HALFLEN + HOUSING_GAP`), close to where `ankle_{SIDE}_Link` attaches, by the foot |

These two placements aren't symmetric, and that's intentional -- it matches
`biped.xml`'s real numbers, not a guess. Its two knee-actuator cylinders
(inside `hip_R_Link`) sit only 0.04-0.08 m from `hip_R_Link`'s own origin
against a 0.35 m thigh reach -- i.e. clustered at the *top* of the leg, right
next to the ab/ad and hip actuators. Its ankle-actuator cylinder (inside
`knee_R_Link`) sits 0.32 m out of a 0.34 m shank reach -- i.e. clustered at
the *bottom*, right by the ankle/foot. So: ab/ad + hip + knee actuators all
bunch together near the top of the leg; the ankle actuator sits alone, near
the foot; only the thigh and shank *struts* (not the actuator housings)
actually span the full leg-segment lengths.

`hip_{SIDE}_Link`'s own origin coincides exactly with where the hip
actuator (one body up, in `abad_{SIDE}_Link`) is drawn -- both sit at
`ABAD_TO_HIP_OFFSET`. Since both housings are coaxial Y-oriented cylinders,
separating them requires offsetting along their *shared long axis* (Y), not
perpendicular to it -- an earlier version of this builder used a small Z
offset here, which barely separated the two cylinders and left them
radially overlapping. `biped.xml` itself stacks its two knee-actuator
cylinders along Y from `hip_R_Link`'s origin (0.04, 0.08 m) for exactly this
reason. `KNEE_AT_HIP_Y_OFFSET = HIP_AT_ABAD_HOUSING_HALFLEN +
KNEE_AT_HIP_HOUSING_HALFLEN + HOUSING_GAP` sizes that offset so the two
coaxial cylinders' end faces clear each other by `HOUSING_GAP`, regardless
of the actuator sizes picked.

- This only moves *where each actuator's mass/housing geometry sits*.
  `act_abad`/`act_hip`/`act_knee`/`act_ankle` each still drive their own
  joint's `<motor>` and `<default>` torque limit (`build_xml` computes
  `limits` straight from `spec.actuators()`, independent of body placement)
  -- see `_abad_body`, `_hip_body`, `_knee_body`, `_ankle_body` in
  `builder.py`.
- `base_Link`'s combined inertial (trunk box + both ab/ad actuators) only
  tracks the shared z-offset (`-hip_z_drop`) via the parallel-axis combiner;
  the lateral (`±hip_y`) and fore-aft (`hip_x`) offsets are dropped -- the
  same z-only simplification used throughout this file.

## Fixed hardware constants (not part of `DesignSpec`)

Like quadruped_xml_builder's `ABD_RADIUS`/`HIP_RADIUS`/`KNEE_RADIUS`/
`HIP_STRUCT_MASS`/etc., these are hardcoded in `builder.py` rather than
searched -- they represent housing/strut/joint properties treated as fixed
hardware, not design-space variables:

| Constant | Value | What it is |
|---|---|---|
| `ABAD_OWN_HOUSING_RADIUS` / `_HALFLEN` | 0.0483 / 0.02725 m | Ab/ad actuator's own housing, a massless decal on `base_Link` |
| `HIP_AT_ABAD_HOUSING_RADIUS` / `_HALFLEN` | 0.0483 / 0.02725 m | Hip actuator housing cylinder, proximally mounted on `abad_{SIDE}_Link` (Rule 3) |
| `KNEE_AT_HIP_HOUSING_RADIUS` / `_HALFLEN` | 0.0483 / 0.016 m | Knee actuator housing cylinder, proximally mounted on `hip_{SIDE}_Link` (Rule 3) |
| `ANKLE_AT_KNEE_HOUSING_RADIUS` / `_HALFLEN` | 0.03 / 0.02725 m | Ankle actuator housing cylinder, proximally mounted on `knee_{SIDE}_Link` (Rule 3) |
| `HOUSING_GAP` | 0.005 m | Gap kept between two adjacent housings, or a housing and the distal end of the body it sits on |
| `KNEE_AT_HIP_Y_OFFSET` | 0.04825 m (derived) | Y-axis offset of the knee housing from `hip_{SIDE}_Link`'s origin, so it clears the coaxial hip housing above it (Rule 3) |
| `THIGH_STRUT_HALF_WIDTH` / `_THICK` | 0.025 / 0.016 m | Thigh strut box cross-section |
| `SHANK_STRUT_HALF_WIDTH` / `_THICK` | 0.0125 / 0.016 m | Shank strut box cross-section |
| `STRUT_DENSITY` | 700 kg/m^3 | Effective density of a hollow carbon/AL strut |
| `TRUNK_DENSITY` | 899.5 kg/m^3 | Effective density incl. electronics/battery, calibrated so the baseline trunk box masses ~12 kg (matches `biped.xml`'s `base_Link`) |
| `JOINT_STRUCT_MASS` | 0.10 kg | Mount/bearing mass added at each actuator housing |
| `FOOT_STRUCT_MASS` | 0.30 kg | `ankle_{SIDE}_Link`'s entire mass -- foot/bracket structure, no actuator of its own (Rule 3) |
| `ABAD_TO_HIP_OFFSET_X` / `_Y` | -0.0683 / 0.0305 m | Fixed mount-bracket offset between the ab/ad and hip actuators |
| `FOOT_X_OFFSET_FRAC` | 0.05 | Foot box centre offset forward of the ankle joint, as a fraction of `foot_length` |
| `FOOT_ANKLE_GAP` | 0.02 m | Vertical gap from the ankle joint down to the top of the foot box |
| `JOINT_DYNAMICS` | see `biped.xml`'s `<default>` | Per-class `damping`/`armature`/`frictionloss` |
| `JOINT_RANGES` | see `biped.xml`'s `<default>`/joints | One canonical `range=` per joint class, mirrored across sides |

`JOINT_RANGES` uses one range per joint *type* for both legs. The real
`biped.xml` has slightly different knee ranges between `R` and `L` (a
hand-tuned CAD/hardware asymmetry) -- this builder treats the design as
bilaterally symmetric, using the right leg's range for both sides.

## Torque limits and actuator picks

Unlike quadruped_xml_builder (which uses `<position>` actuators with
`kp=100` and per-joint `forcerange`), this builder emits `<motor>`
(direct-torque) actuators inside `<default>` classes -- matching
`biped.xml` and what `My_Controller` actually commands. Each joint class's
`ctrlrange`/`forcerange` is `±peak_torque` of the `DesignSpec` actuator
picked for that joint *type* (`act_abduction`/`act_hip`/`act_knee`/
`act_ankle`), mirrored across both legs -- same one-actuator-type-per-joint
mirroring quadruped_xml_builder uses for `act_abduction`/`act_hip`/
`act_knee`.
