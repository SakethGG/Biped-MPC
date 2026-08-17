# Biped design parameters (`DesignSpec`, `generate.py`)

Search bounds (`DesignBounds`) and the baseline used by `generate.py`'s
example build (read back from `models/tron/biped.xml`'s real dimensions
where a direct equivalent exists). `DesignSpec`/`DesignBounds` live in
`generate.py` itself (formerly a separate `spec.py`). Biped analogue of
quadruped_xml_builder's `DESIGN_PARAMETERS.md`.

`build_xml` produces the ACTUATOR-CLUSTER topology (matching
`design_biped.xml`/`new_biped.xml`), not `models/tron/biped.xml`'s own
simple joint chain -- see `GEOMETRY_RULES.md`. `hip_x`/`hip_y`/`hip_z_drop`
(the old chain topology's abad-joint mount offset) have no role in this
topology and are gone from `DesignSpec`.

## Naming convention

Every clearance/offset field is named `clearance_{source}2{target}_{axis}`
-- "clearance from `source` to `target`, along `axis`" -- where `source`/
`target` are the two bodies the offset sits between (chained down the leg:
`trunk` -> `abadAct` -> `hipAct` -> `kneeAct` -> `thigh` -> ... -> `foot`).
E.g. `clearance_trunk2abadAct_z` is the abad housing's z-offset off the
trunk; `clearance_abadAct2hipAct_x` is the hip housing's x-offset off the
abad housing.

`DesignSpec`'s fields split into two tiers (see its class docstring in
`generate.py`):

- **Free fields** -- swept by `DesignBounds`/`vector()`/`from_vector()`, the
  table below.
- **Fixed-constant fields** -- a Python default on the `DesignSpec`
  dataclass, not part of `DesignBounds`/`vector()`/`sample()`. Mostly
  zero-valued clearances; see the second table below.
- **Not `DesignSpec` fields at all** -- `thigh_width`/`thigh_thickness`/
  `shank_width`/`shank_thickness`/`foot_width`/`foot_height` used to be spec
  fields but were removed once nothing needed them to vary independently;
  they're now fixed `builder.py` constants (`THIGH_BOX_HALF`/
  `SHANK_BOX_HALF`/`FOOT_BOX_HALF`), see the note after the second table.

## Free fields

| Field | Bounds (`DesignBounds`) | Baseline (`generate.py`) | Free in `build_xml`? |
|---|---|---|---|
| `trunk_length` | (0.20, 0.40) m | 0.27 m | Yes |
| `trunk_width` | (0.18, 0.34) m | 0.26 m | Yes, but must clear `2 * abad_actuator_radius` |
| `trunk_thickness` | (0.12, 0.26) m | 0.19 m | Yes |
| `clearance_trunk2abadAct_z` | (0.02, 0.08) m | 0.050 m | Yes |
| `clearance_abadAct2hipAct_x` | (0.005, 0.04) m | 0.020 m | Yes |
| `clearance_abadAct2hipAct_y` | (0.005, 0.03) m | 0.010 m | Yes, but must satisfy the hip-offset formula's sqrt-term guard |
| `clearance_hipAct2kneeAct_y` | (0.0, 0.03) m | 0.01 m | Yes |
| `thigh_length` | (0.18, 0.45) m | 0.2 m | Yes -- scales along a fixed bend direction, doesn't change it |
| `shank_length` | (0.22, 0.45) m | 0.3 m | Yes -- same |
| `foot_length` | (0.14, 0.26) m | 0.2 m | Yes -- foot box's x-extent only; width/height are NOT a `DesignSpec` field, see `FOOT_BOX_HALF` below |
| `body_height` | (0.30, 0.85) m | 0.767 m | Yes -- sets `base_Link`'s spawn Z directly, no reachability check |
| `act_abduction` | catalog index or name | `3` (`"TMotor-AK80-9"`, 24.00 Nm) | Yes |
| `act_hip` | catalog index or name | `3` (`"TMotor-AK80-9"`, 24.00 Nm) | Yes |
| `act_knee` | catalog index or name | `3` (`"TMotor-AK80-9"`, 24.00 Nm) | Yes |
| `act_ankle` | catalog index or name | `4` (`"TMotor-AK80-6"`, 12.00 Nm) | Yes |

## Fixed-constant fields

Each defaults on the `DesignSpec` dataclass itself -- omit them at
construction time to get the value below. Not part of `DesignBounds` /
`vector()` / `sample()`, but every field below IS read by `build_xml`.

| Field | Default | Used by `build_xml` today? |
|---|---|---|
| `clearance_trunk2abadAct_x` | 0.0 m | Yes -- `x_abad` offset |
| `clearance_trunk2abadAct_y` | 0.0 m | Yes -- inside `yyy` |
| `clearance_abadAct2hipAct_z` | 0.0 m | Yes -- `z_hip` offset |
| `clearance_hipAct2kneeAct_x` | 0.0 m | Yes -- `x_knee` offset |
| `clearance_hipAct2kneeAct_z` | 0.0 m | Yes -- `z_knee` offset |
| `clearance_kneeAct2thigh_x` | 0.0 m | Yes -- `x_thigh` offset |
| `clearance_kneeAct2thigh_y` | 0.0 m | Yes -- added on top of `THIGH_BOX_HALF[1]` in the `y_thigh` offset |
| `clearance_kneeAct2thigh_z` | 0.0 m | Yes -- `z_thigh` offset |
| `clearance_shank2ankleAct_x` | 0.0 m | Yes -- added to `spec.shank_length * SHANK_DIR` |
| `clearance_shank2ankleAct_y` | 0.0 m | Yes -- same |
| `clearance_shank2ankleAct_z` | 0.0 m | Yes -- same |
| `clearance_ankleAct2foot_x` | 0.0 m | Yes -- `foot_{SIDE}_Link`'s own `pos` relative to `ankle_{SIDE}_act` (the foot box/site's position *inside* `foot_{SIDE}_Link` is still `FOOT_X_OFFSET_FRAC`/`FOOT_ANKLE_GAP`, unrelated `builder.py` constants) |
| `clearance_ankleAct2foot_y` | 0.0 m | Yes -- same |
| `clearance_ankleAct2foot_z` | 0.03 m | Yes -- same; must stay positive (foot sits below the ankle actuator) |

`foot_width`/`foot_height`/`thigh_width`/`thigh_thickness`/`shank_width`/
`shank_thickness` are NOT `DesignSpec` fields at all (not even
fixed-constant ones) -- the foot box's y/z cross-section is `builder.py`'s
`FOOT_BOX_HALF = (0.035, 0.015)` (half-width, half-height), the shank box's
is `SHANK_BOX_HALF = (0.0125, 0.016)`, and
and `THIGH_BOX_HALF[1]` (0.016 m -- the thigh box's local-Y half-extent,
which stays aligned with global Y since `THIGH_BOX_QUAT` only rotates about
Y) replaces the old decoupled `spec.thigh_width`/`THIGH_WIDTH_GAP` gap-fill
width in both the `y_knee` and `y_thigh` offset formulas (`THIGH_BOX_HALF[0]`,
the box's local-X half-extent, lies within the sagittal plane and plays no
role in these Y-axis formulas) -- but since it's a HALF-extent, `y_knee`
uses `2*THIGH_BOX_HALF[1]` (the box's FULL Y-extent, matching what the old
full-width `thigh_width` field meant) while `y_thigh` and the thigh's own
tangent-cylinder half-length use `THIGH_BOX_HALF[1]` directly. Only
`foot_length` (the foot box's x-extent) stays spec-controlled; the thigh
box's own length is computed per-spec, not from
a `THIGH_BOX_HALF` entry.

**`z_abad` discrepancy**: the source notes for the abad z-offset formula
say `z_abad = -(trunk_length/2 + r_abad + clearance_trunk2abadAct_z)`, but
`build_xml` uses `trunk_thickness/2`, not `trunk_length/2` -- kept as-is
since it matches the trunk box's actual z half-extent (`spec.trunk_thickness
/ 2`) and reproduces `design_biped.xml`'s real geometry to 6 decimal places;
`trunk_length/2` looks like a copy/paste slip in the notes rather than an
intended change. Flag if `trunk_length` was actually meant here.

`clearance_shank2ankleAct_*`/`clearance_ankleAct2foot_*` cover the actuator
between the shank and the foot -- i.e. **the ankle actuator** (`act_ankle`).
An earlier draft of these notes called it `footAct`, which collides with the
foot *link*'s own name; `ankleAct` is used instead throughout `DesignSpec`.

## Actuator placement offsets (`design_biped.xml`, now also `build_xml`'s topology)

`design_biped.xml` (and its byte-identical twin `new_biped.xml`) is a
hand-built file predating `builder.py`'s current version -- `build_xml` now
produces this SAME actuator-cluster topology, parametrized (see
`GEOMETRY_RULES.md`, and `builder.py`'s module docstring). It places a
genuine actuator CLUSTER per leg -- `abad_{R,L}_act` -> `hip_{R,L}_act` ->
`knee_{R,L}_act`, each a child of the previous one, all three near the top
of the leg -- then continues into a `thigh_{R,L}_Link`, `shank_{R,L}_Link`
(the belt-driven knee joint lives here), and a combined `ankle_{R,L}_act`
(ankle actuator + foot) at the very end. At the baseline `DesignSpec` in
`generate.py`, all three cluster actuators use the catalog entry
`MotorU12_framed+WPG:N=10.50` (radius 0.044972 m, halflen 0.02725 m, mass
2.247312 kg, peak torque 79.82 Nm); the ankle uses
`MotorMN8014_framed+CPG:N=4.00` (radius 0.03 m, halflen 0.02725 m, mass
0.7 kg -- catalog said this wasn't queried, so it reuses biped.xml's
combined ankle-link mass instead -- peak torque 19.97 Nm). Running
`generate.py` at this baseline reproduces `design_biped.xml`'s
`abad_{R,L}_act`/`hip_{R,L}_act`/`knee_{R,L}_act`/`thigh_{R,L}_Link`
positions to 6 decimal places; `shank_{R,L}_Link`/`ankle_{R,L}_act` land
~2mm further out (see `GEOMETRY_RULES.md`'s note on `thigh_length`/
`shank_length` scaling a fixed direction, not a re-derived one).

| Body | Parent | pos (rel. parent) | quat |
|---|---|---|---|
| `abad_R_act` | `base_Link` | `0.027250 -0.085028 -0.189972` | -- |
| `hip_R_act` | `abad_R_act` | `-0.092222 0.029213 0.000000` | -- |
| `knee_R_act` | `hip_R_act` | `0.000000 -0.086500 0.000000` | -- |
| `thigh_R_Link` | `knee_R_act` | `0.000000 0.043250 0.000000` | -- |
| `shank_R_Link` | `thigh_R_Link` | `-0.200752 0.0 -0.286703` | `0.939692 0 -0.342022 0` |
| `ankle_R_act` | `shank_R_Link` | `-0.017221 0.0 -0.343079` | `0.939681 0 0.342052 0` |

(Left side mirrors in Y; `hip_L_act`/`knee_L_act` flip sign on their own Y
offset, `thigh_L_Link` too -- `shank_*_Link`/`ankle_*_act` are identical both
sides, see below.)

**`abad_R_act`/`hip_R_act`/`knee_R_act`** -- positioned by the clearance-based
x/y/z offset formulas (chained: abad relative to `base_Link`, hip relative
to the abad housing, knee relative to the hip housing):
```
x_abad = thickness_abad/2
yyy = trunk_width/2 - r_abad
y_abad = ±yyy
z_abad = -(trunk_thickness/2 + r_abad + 0.050)          # clearance1

x_hip = -(thickness_abad/2 + r_hip + 0.020)              # clearance2
y_hip = ±(sqrt((yyy + 0.010)^2 - r_hip^2) - thickness_hip)  # clearance3
z_hip = 0

x_knee = 0
y_knee = ±(thickness_hip/2 + width_thigh + thickness_knee/2)
z_knee = 0
```
`width_thigh = 0.032` m (`2 * THIGH_STRUT_HALF_THICK`), `trunk_width = 0.26`
m, `trunk_thickness = 0.19` m -- same constants as `builder.py`'s
`_abad_housing_pos`/`_hip_housing_offset`/`_knee_housing_offset`, though this
file predates and isn't generated by that code (the numbers were derived by
hand the same way, then hardcoded here).

**`thigh_R_Link`** -- a cylinder (radius = knee actuator's radius, length =
the full hip/knee-actuator gap) plus a cuboid (dimensions copied from
`biped.xml`'s original thigh box, `0.025 0.016 0.125`) whose small end face
sits tangent to that cylinder's side, extending along `biped.xml`'s
hip->knee direction (X-Z plane only). No explicit `<inertial>` -- mass/COM/
inertia auto-derived by MuJoCo from geom density x volume (aluminum,
2700 kg/m^3; visual-class geoms use `density="0"` so they don't double-count
against the collision geoms).

**`shank_R_Link`** -- attached using `biped.xml`'s exact original
`hip_R_Link -> knee_R_Link` pos/quat, with the Y-component zeroed for both
legs. Carries the belt-driven `knee_{R,L}_Joint` (axis `0 -1 0`, range
`(-1.21357, 1.17267)` R / `(-0.872665, 1.36136)` L -- `biped.xml`'s own
asymmetric ranges, not a symmetrized one). Its own geometry/inertial is a
placeholder, reused verbatim from `biped.xml`'s `knee_R_Link`/`knee_L_Link`
box (`size="0.0125 0.016 0.15"`, mass 1.0 kg) -- swap in real shank
dimensions when available.

**`ankle_R_act`** -- attached using `biped.xml`'s exact `knee_R_Link ->
ankle_R_Link` pos/quat. Combines the ankle actuator housing (cylinder,
catalog radius/halflen) and the foot (cuboid, `size="0.1 0.035 0.015"`,
copied verbatim from `biped.xml`'s foot box) in one body, carrying the
`ankle_{R,L}_Joint` (axis `0 -1 0`, range `(-0.791917, 1.30899)`, same both
sides) and the `foot_R`/`foot_L` sensor site.

This table is kept in sync with `design_biped.xml`/`new_biped.xml` -- update
it whenever an offset changes there.

This table is kept in sync with `design_biped.xml` -- update it whenever an
offset changes there.

Every field above is read directly off `models/tron/biped.xml`'s real
geometry (trunk box, ab/ad mount offset, hip->knee and knee->ankle bone
lengths, foot box) -- see the comments in `generate.py`'s `DesignSpec(...)`
block for exactly which XML attribute each one came from. `body_height =
0.5795` is chosen so `base_z = hip_z_drop + body_height = 0.767`, the same
spawn height as `biped.xml`'s `base_Link`, for direct side-by-side
comparison in the viewer.

Unlike quadruped_xml_builder's `build_xml_v2`, **no field here is silently
overridden** -- `trunk_thickness`, `hip_x`, and `hip_y` all stay as the
caller set them (see GEOMETRY_RULES.md for why: the ab/ad mount offset on a
biped pelvis isn't derived from trunk dimensions the way the quadruped's
flush-against-the-trunk-face rule works). `body_height` is the only field
that can be silently adjusted, and only via clipping inside
`home_leg_angles` when it falls outside the reachable band (Rule 2) -- at
the baseline above it's already inside that band with a small margin (knee
home angle comes out to ~-67.2 deg, close to but inside the knee's real
-69.5 deg range floor, since 0.5795 m is a deep, near-full-extension stance).

At the baseline above, the picked actuators are:

- `act_abduction`/`act_hip`/`act_knee` = `"MotorU12_framed+WPG:N=10.50"`,
  peak_torque = 79.82 Nm (biped.xml's real ab/ad/hip/knee torque limit is
  80 Nm).
- `act_ankle` = `"MotorMN8014_framed+CPG:N=4.00"`, peak_torque = 19.97 Nm
  (biped.xml's real ankle torque limit is 20 Nm).

`ACTUATOR_CATALOG` now defaults to `catalogs/actuators.json` (13 entries),
not the `catalogs/compact/` motor x gearbox x ratio cross-product (84
entries) -- see `actuator.py`'s module docstring. Both entries above were
appended to `actuators.json` specifically to carry these two torque values
(neither the original 11-entry catalog nor `catalogs/compact/`'s generic
BLDC/gearbox data had a matching part) -- see that file's `"notes"` fields
for exactly which numbers are real datasheet values vs. unvalidated
placeholders (peak_speed and cost, mainly).

Actuators are referenced by **name**, not index, in `generate.py` --
indices shift any time `actuators.json` gains or loses an entry, names
don't. An int index still works (`spec.actuators()` accepts either), but
isn't recommended for anything you intend to keep working after the next
catalog edit. `set_active_catalog("compact", ...)` (in `actuator.py`) swaps
back to the `catalogs/compact/` cross-product if you want that design space
instead -- indices there depend on motor/gearbox list order and
`ratios_per_type`, printed by iterating `ACTUATOR_CATALOG` after the swap.

## Reused design-space fields, not `DesignSpec` fields

`per_joint_tau_max()` returns the 8-vector torque clip in `[R,L] x
(abad,hip,knee,ankle)` order, mirroring quadruped_xml_builder's
`per_joint_tau_max()` (`[FR,FL,RR,RL] x (hip,thigh,calf)`) -- for an external
controller/optimizer to consume, not used by anything in this folder.

`Constraints`/`CostWeights` from quadruped_xml_builder's `spec.py` were
**not** ported -- they belonged to a design-search harness
(`optimize_design.py`/`cmaes_design_search.py`) that isn't present in either
repo (see quadruped_xml_builder's own docs, which reference those files
without shipping them). Add them back here if/when a biped design-search
loop is actually built.