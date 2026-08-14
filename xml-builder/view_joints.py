"""Launch MuJoCo viewer with the base fixed and joint sliders active.

Usage:
    python view_joints.py [path/to/xml] [spawn_height_offset]

Injects a position actuator for every revolute joint so that the
viewer's Control tab shows a slider per joint.

The base's freejoint is stripped (so it stays fixed while sliders move the
legs), which means its spawn height lives in the base body's static `pos=`
attribute rather than in `qpos`. For files using the old straight-chain
topology (`abad_R_Link`/`knee_R_Link`/`ankle_R_Link` bodies present), the
base is re-anchored to `hip_z_drop + thigh_length + shank_length +
spawn_height_offset` (offset defaulting to `DEFAULT_SPAWN_OFFSET`, override
via the second CLI argument). For the current actuator-cluster topology
(`design_biped.xml`/`new_biped.xml`/`design.xml`), those anchor bodies don't
exist, so this falls through to the source XML's own authored `base_Link`
pos -- already correct, since `DesignSpec.body_height` sets it directly.

Direct port of quadruped_xml_builder's view_joints.py -- only the body names
used to re-derive leg lengths changed (`FR_calf`/`FR_foot` -> `knee_R_Link`/
`ankle_R_Link`, plus `abad_R_Link` for the trunk-to-hip drop that the
quadruped doesn't have).
"""
import os
import re
import sys
from pathlib import Path
import mujoco
import mujoco.viewer

_default = str(Path(__file__).parent / "design.xml")
DEFAULT_SPAWN_OFFSET = 0.02   # [m] ground clearance above hip_z_drop + thigh + shank reach
xml_path = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else _default)

# chdir so relative mesh asset paths resolve when loading from string
os.chdir(os.path.dirname(xml_path))

# MjSpec.from_file resolves <include> tags; to_xml() returns fully expanded XML
spec = mujoco.MjSpec.from_file(xml_path)
xml = spec.to_xml()

# Remove freejoint -- to_xml() serializes it as <joint name="root" type="free".../>
xml = re.sub(r'<joint\b[^>]*type="free"[^/]*/>', '', xml)

# Find nq without the freejoint (need it to trim keyframe qpos)
_tmp = mujoco.MjModel.from_xml_string(
    re.sub(r'<keyframe>.*?</keyframe>', '', xml, flags=re.DOTALL)
)
n_q = _tmp.nq

# Trim leading freejoint values from keyframe qpos; drop ctrl (actuators change)
def _trim_qpos(m):
    vals = m.group(1).split()
    excess = len(vals) - n_q
    if excess > 0:
        vals = vals[excess:]
    return f'qpos="{" ".join(vals)}"'

xml = re.sub(r'\bqpos="([^"]*)"', _trim_qpos, xml)
xml = re.sub(r'\bctrl="[^"]*"', '', xml)

# Drop any existing actuators -- they conflict with the viewer's injected ones
xml = re.sub(r'<actuator>.*?</actuator>', '', xml, flags=re.DOTALL)

# Discover hinge joints, inject position actuators for Control tab sliders
_model = mujoco.MjModel.from_xml_string(xml)
actuator_lines = []
for i in range(_model.njnt):
    jnt = _model.joint(i)
    if jnt.type == mujoco.mjtJoint.mjJNT_HINGE:
        lo, hi = _model.jnt_range[i]
        actuator_lines.append(
            f'  <position name="{jnt.name}" '
            f'joint="{jnt.name}" kp="10" '
            f'ctrlrange="{lo:.4f} {hi:.4f}"/>'
        )

if actuator_lines:
    actuator_block = (
        "<actuator>\n"
        + "\n".join(actuator_lines)
        + "\n</actuator>"
    )
    xml = xml.replace("</mujoco>", actuator_block + "\n</mujoco>")

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Re-anchor the (fixed, since its freejoint was stripped above) base to
# hip_z_drop + thigh_length + shank_length + offset -- ONLY for the old
# straight-chain topology (abad_R_Link/knee_R_Link/ankle_R_Link, pure -Z
# drops at each body). The current actuator-cluster topology
# (design_biped.xml/new_biped.xml/design.xml) sets base_Link's spawn height
# directly from DesignSpec.body_height, and its body offsets aren't pure Z
# drops (the abad/hip/knee cluster has X/Y components too) -- so that sum
# wouldn't give a meaningful answer here even if the anchor bodies existed.
# Falls through to the source XML's own authored pos=, which is already
# correct for this topology.
offset = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_SPAWN_OFFSET
base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
_leg_anchor_names = ("abad_R_Link", "knee_R_Link", "ankle_R_Link")
_has_leg_anchors = all(
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0
    for name in _leg_anchor_names
)
if base_id >= 0 and _has_leg_anchors:
    hip_z_drop = -model.body("abad_R_Link").pos[2]
    thigh_length = -model.body("knee_R_Link").pos[2]
    shank_length = -model.body("ankle_R_Link").pos[2]
    spawn_z = hip_z_drop + thigh_length + shank_length + offset
    model.body_pos[base_id, 2] = spawn_z
    print(
        f"Spawn height: hip_z_drop={hip_z_drop:.4f} + thigh={thigh_length:.4f} "
        f"+ shank={shank_length:.4f} + offset={offset:.4f} = {spawn_z:.4f} m"
    )
else:
    print(f"Using base_Link's authored spawn height: {model.body_pos[base_id, 2]:.4f} m")

# Apply home keyframe if present (gives standing pose)
kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if kid >= 0:
    mujoco.mj_resetDataKeyframe(model, data, kid)
else:
    mujoco.mj_resetData(model, data)

mujoco.mj_forward(model, data)

print(f"Loaded: {xml_path}")
print(f"Joints:    {[model.joint(i).name for i in range(model.njnt)]}")
print(f"Actuators: {[model.actuator(i).name for i in range(model.nu)]}")

mujoco.viewer.launch(model, data)