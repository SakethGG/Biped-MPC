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
topology and are gone from `DesignSpec`; `clearance_abad_z`/
`clearance_hip_x`/`clearance_hip_y` replace them -- the three clearance
constants in the abad/hip actuator-housing placement formulas.

| Field | Bounds (`DesignBounds`) | Baseline (`generate.py`) | Free in `build_xml`? |
|---|---|---|---|
| `trunk_length` | (0.20, 0.40) m | 0.27 m | Yes |
| `trunk_width` | (0.18, 0.34) m | 0.26 m | Yes, but must clear `2 * abad_actuator_radius` (Rule 1) |
| `trunk_thickness` | (0.12, 0.26) m | 0.19 m | Yes |
| `clearance_abad_z` | (0.02, 0.08) m | 0.050 m | Yes |
| `clearance_hip_x` | (0.005, 0.04) m | 0.020 m | Yes |
| `clearance_hip_y` | (0.005, 0.03) m | 0.010 m | Yes, but must satisfy Rule 2's sqrt-term guard |
| `thigh_length` | (0.22, 0.45) m | 0.3522218416 m | Yes -- scales along a fixed bend direction, doesn't change it |
| `shank_length` | (0.22, 0.45) m | 0.3435109359 m | Yes -- same |
| `foot_length` | (0.14, 0.26) m | 0.2 m | Yes |
| `foot_width` | (0.05, 0.10) m | 0.07 m | Yes |
| `foot_height` | (0.015, 0.05) m | 0.03 m | Yes |
| `body_height` | (0.30, 0.85) m | 0.767 m | Yes -- sets `base_Link`'s spawn Z directly, no reachability check |
| `act_abduction` | catalog index or name | `"MotorU12_framed+WPG:N=10.50"` | Yes |
| `act_hip` | catalog index or name | `"MotorU12_framed+WPG:N=10.50"` | Yes |
| `act_knee` | catalog index or name | `"MotorU12_framed+WPG:N=10.50"` | Yes |
| `act_ankle` | catalog index or name | `"MotorMN8014_framed+CPG:N=4.00"` | Yes |

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