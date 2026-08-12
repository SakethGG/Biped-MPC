# Biped design parameters (`DesignSpec`, `spec.py`)

Search bounds (`DesignBounds`) and the baseline used by `generate.py`
(read back from `models/tron/biped.xml`'s real dimensions where a direct
equivalent exists). Biped analogue of quadruped_xml_builder's
`DESIGN_PARAMETERS.md`.

| Field | Bounds (`DesignBounds`) | Baseline (`generate.py`) | Free in `build_xml`? |
|---|---|---|---|
| `trunk_length` | (0.20, 0.40) m | 0.27 m | Yes |
| `trunk_width` | (0.18, 0.34) m | 0.26 m | Yes |
| `trunk_thickness` | (0.12, 0.26) m | 0.19 m | Yes |
| `hip_x` | (0.00, 0.08) m | 0.03415 m | Yes |
| `hip_y` | (0.05, 0.14) m | 0.0817 m | Yes, but must clear `MIN_HIP_Y = ABAD_HOUSING_RADIUS + 0.01` (Rule 1) |
| `hip_z_drop` | (0.10, 0.28) m | 0.1875 m | Yes |
| `thigh_length` | (0.22, 0.45) m | 0.3522218416 m | Yes |
| `shank_length` | (0.22, 0.45) m | 0.3435109359 m | Yes |
| `foot_length` | (0.14, 0.26) m | 0.2 m | Yes |
| `foot_width` | (0.05, 0.10) m | 0.07 m | Yes |
| `foot_height` | (0.015, 0.05) m | 0.03 m | Yes |
| `body_height` | (0.30, 0.85) m | 0.5795 m | Yes, but must satisfy Rule 2 (or gets silently clipped) |
| `act_abduction` | `(0, len(catalog)-1)` int | 6 | Yes |
| `act_hip` | `(0, len(catalog)-1)` int | 6 | Yes |
| `act_knee` | `(0, len(catalog)-1)` int | 6 | Yes |
| `act_ankle` | `(0, len(catalog)-1)` int | 51 | Yes |

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

- `act_abduction`/`act_hip`/`act_knee` = index 6 =
  `MotorU12_framed+WPG:N=10.50`, peak_torque ~= 79.82 Nm (biped.xml's real
  ab/ad/hip/knee torque limit is 80 Nm).
- `act_ankle` = index 51 = `MotorMN8014_framed+CPG:N=4.00`, peak_torque ~=
  19.97 Nm (biped.xml's real ankle torque limit is 20 Nm).

These indices are specific to the *default* `ACTUATOR_CATALOG` built from
`catalogs/compact/` at import time (6 motors x 4 gearbox topologies x 3
subsampled ratios each = 84 entries as of this writing). If you edit the
catalogs, the indices will point at different actuators -- re-run with
`--list`-style inspection (print `ACTUATOR_CATALOG[i].name` /
`.peak_torque`) or search for the closest match to your target torque
before hardcoding a new index, the same way these two were picked.

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
