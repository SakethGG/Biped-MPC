# Biped XML Builder

Standalone, single-folder package for generating a parametric biped MJCF
(MuJoCo XML) from a `DesignSpec` -- trunk dimensions, leg segment lengths,
foot dimensions, and per-joint actuator picks. Self-contained: no dependency
on any code outside this folder.

This is a port of `/Users/sakethvegesna/Projects/quadruped_xml_builder`'s
design (same `catalogs/` + `actuator.py` + `spec.py` + `builder.py` +
`generate.py` + `view_joints.py` pipeline) from a 4-leg/3-joint quadruped
to the 2-leg/4-joint (ab/ad, hip, knee, ankle) TRON1 biped used by this
repo. A generated file matches `models/tron/biped.xml`'s body/joint/actuator
naming, `<default>` classes, sensors, freejoint, and keyframe structure
closely enough to be a drop-in `model.xml_path` for
`config/tron/my_controller.yaml` -- see GEOMETRY_RULES.md for exactly what
is and isn't reproduced byte-for-byte.

## Quick start

```bash
python3 -m venv .venv && source .venv/bin/activate   # or use an existing env
pip install mujoco numpy
```

Generate a design:

```bash
python generate.py my_design.xml
```

Reads the `DesignSpec(...)` block at the top of `generate.py`, builds the
MJCF, and writes it to `my_design.xml`. Prints which actuator (motor +
gearbox + ratio) got picked for each joint. Edit the numbers in that block
to change the design -- bounds are documented in `DESIGN_PARAMETERS.md`.

View it:

```bash
python view_joints.py my_design.xml
```

Opens the MuJoCo viewer with the base fixed in place and a slider per joint
(Control tab) so you can pose each leg by hand.

To use a generated design with the actual controller, point
`config/tron/my_controller.yaml`'s `model.xml_path` /
`model.auxiliary_xml_path` at the output file and run
`./build/apps/main t y` as usual (see `commands.md` at the repo root).

## Files

| File | What it does |
|---|---|
| `generate.py` | Example script: builds one `DesignSpec` into an XML file. Edit this to change the design. |
| `builder.py` | `build_xml(spec)` -- the actual MJCF generator. |
| `spec.py` | `DesignSpec` dataclass -- the design variables. |
| `actuator.py` | Builds `ACTUATOR_CATALOG` (motor x gearbox x ratio) from `catalogs/`. Unmodified port -- generic motor/gearbox physics, no notion of leg count. |
| `catalogs/` | Motor + COMPAct gearbox data tables `actuator.py` reads. Copied verbatim from quadruped_xml_builder. |
| `view_joints.py` | MuJoCo viewer with per-joint sliders. |
| `design.xml` | Example output of `generate.py` at the baseline `DesignSpec` (TRON1-derived dimensions). |
| `GEOMETRY_RULES.md` | The placement rules `build_xml` enforces, plus every fixed hardware constant (housing radii, strut cross-sections, joint dynamics/ranges) it uses in place of `DesignSpec` fields. |
| `DESIGN_PARAMETERS.md` | Bounds and baseline for every `DesignSpec` field. |

## What's different from quadruped_xml_builder

- **2 legs, 4 joints each** (`abad`/`hip`/`knee`/`ankle`), not 4 legs x 3
  joints. `LEG_SIDES = ("R", "L")`, mirrored only in `y`.
- **`<motor>` actuators**, not `<position>` -- `My_Controller` commands
  torque directly, so the generated file's actuator block has to match
  `biped.xml`'s convention (per-class `ctrlrange`/`forcerange` from the
  picked actuator's `peak_torque`, not a per-instance PD gain).
- **Body/joint/class naming matches `biped.xml` exactly**
  (`abad_R_Link`/`abad_R_Joint`/`abad_R_motor`, classes `abad`/`hip`/`knee`/
  `ankle`, `<freejoint name="root"/>`, single `home` keyframe with no `ctrl`
  attribute) so a generated file needs zero changes to
  `sim/src/models/TronBipedSpec.cpp`.
- **No CAD pre-bend quats.** `biped.xml`'s knee/ankle bodies carry ~40 deg
  hand-tuned rotations baked into the link geometry (real hardware
  cable-routing/linkage artifacts). This builder's legs hang straight down
  each parent's local -z axis, like quadruped_xml_builder's
  `FR_thigh`/`FR_calf` capsules -- simpler, fully general, and the
  home-pose 2-link IK (`home_leg_angles`) still works unmodified, but a
  generated file's home-pose joint angles are only *structurally* similar
  to `biped.xml`, not numerically identical. See the module docstring at
  the top of `builder.py` and GEOMETRY_RULES.md.
- **Proximal actuator mounting**, matching `biped.xml`: every actuator's
  housing/mass sits one body *up* the chain from the joint it drives, not
  co-located with it -- the hip actuator's housing lives on
  `abad_{SIDE}_Link`, the knee actuator's on `hip_{SIDE}_Link`, the ankle
  actuator's on `knee_{SIDE}_Link`, and the ab/ad actuator's own housing is
  a massless decal directly on `base_Link` (nothing sits further up the
  chain to proximally mount it on). `ankle_{SIDE}_Link` ends up with no
  actuator of its own, just foot structure. This was verified by rotating
  every housing cylinder's own axis through its actual `quat` in
  `biped.xml`: they all point along the rotation axis of the joint they
  *drive*, not the joint of the body they sit in. Each joint still gets its
  own `<motor>`/torque limit from its own `DesignSpec` actuator pick; only
  the housing geometry's body changed. See GEOMETRY_RULES.md Rule 3.
- **Baseline `DesignSpec` in `generate.py` matches `biped.xml`'s geometry
  exactly** (trunk box, hip offsets, thigh/shank bone lengths, foot box, and
  a `body_height` chosen so the spawn height comes out to the same 0.767 m)
  so the two files are directly comparable side by side.
- **Extra ankle joint's home angle** is solved as `theta_ankle =
  -(theta_hip + theta_knee)` (keeps the foot flat), on top of the same
  `home_leg_angles` 2-link IK quadruped_xml_builder uses for hip/knee.
- **`catalogs/`, `actuator.py` unchanged.** Neither file has any notion of
  how many legs a robot has -- reused verbatim.
- **`Constraints`/`CostWeights` dropped from `spec.py`.** They belonged to
  an external design-search harness not present in quadruped_xml_builder
  either (see that project's own docs) -- not worth carrying dead weight
  for a harness this repo doesn't have.

## Design-space notes

`catalogs/compact/` ships generic BLDC-motor/planetary-gearbox data (COMPAct,
Singh et al. ICRA 2026) -- it isn't tuned to TRON1's actual actuators, so a
generated design's per-joint masses run heavier than the hand-measured
`biped.xml` (the catalog's motor+gearbox combos are chunkier than TRON1's
real actuators). The baseline `DesignSpec` in `generate.py` was chosen to
match `peak_torque` (80 Nm / 20 Nm) rather than mass -- see
`DESIGN_PARAMETERS.md` for the exact picks. Swap in
`set_active_catalog("catalog", actuators_json=...)` with a custom
`catalogs/actuators.json` entry if you want a closer per-joint mass match.
