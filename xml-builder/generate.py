"""Build a biped MJCF from a custom DesignSpec.

Usage:
    python generate.py [output_path.xml]

Edit the `spec = DesignSpec(...)` block below with your own numbers, then run.
See DESIGN_PARAMETERS.md for bounds/defaults and GEOMETRY_RULES.md for the
fixed hardware constants `builder.build_xml` uses alongside these fields.
"""
import sys

from actuator import ACTUATOR_CATALOG
from builder import build_xml
from spec import DesignSpec

# --- edit these --------------------------------------------------------
# Every geometric field below is read directly from models/tron/biped.xml so
# the two files are exactly comparable side by side:
#   - trunk_length/width/thickness = 2x biped.xml's base_Link box <geom size>
#   - hip_x/hip_y/hip_z_drop       = biped.xml's abad_R_Link <body pos>
#   - thigh_length                 = |knee_R_Link <body pos>| (hip->knee bone)
#   - shank_length                 = |ankle_R_Link <body pos>| (knee->ankle bone)
#   - foot_length/width/height     = 2x biped.xml's foot <geom size>
#   - body_height                  = biped.xml base_z (0.767) - hip_z_drop (0.1875),
#                                     so base_z below comes out to exactly 0.767 m,
#                                     the same spawn height as biped.xml.
#
# It will still NOT be byte-identical to biped.xml: this builder uses straight
# leg-link chains (no CAD pre-bend quats), so home-pose joint angles differ
# from the hand-tuned original (deeper hip/knee bend here, since biped.xml's
# pre-bent knee body reaches the same depth with less joint rotation) even
# though every geometric input above, and the body/joint/actuator naming,
# default classes, sensors, and overall structure, match exactly.
spec = DesignSpec(
    trunk_length=0.27,          # 2 * 0.135 m
    trunk_width=0.26,           # 2 * 0.13 m
    trunk_thickness=0.19,       # 2 * 0.095 m
    hip_x=0.03415,
    hip_y=0.0817,
    hip_z_drop=0.1875,
    thigh_length=0.3522218416,  # |(-0.200752, -0.0395, -0.286703)|
    shank_length=0.3435109359,  # |(-0.017221, 0, -0.343079)|
    foot_length=0.2,            # 2 * 0.1 m
    foot_width=0.07,            # 2 * 0.035 m
    foot_height=0.03,           # 2 * 0.015 m
    body_height=0.5795,         # 0.767 - 0.1875, so base_z matches biped.xml exactly
    act_abduction=6,            # MotorU12_framed+WPG:N=10.50, peak_torque=79.82 (biped.xml: 80)
    act_hip=6,                  # same -- biped.xml uses one torque class for ab/ad + hip + knee
    act_knee=6,                 # same
    act_ankle=51,                # MotorMN8014_framed+CPG:N=4.00, peak_torque=19.97 (biped.xml: 20)
)
# -------------------------------------------------------------------------

n_cat = len(ACTUATOR_CATALOG)
if not all(0 <= i < n_cat for i in (spec.act_abduction, spec.act_hip, spec.act_knee, spec.act_ankle)):
    raise SystemExit(f"actuator index out of range: catalog has {n_cat} entries")

out_path = sys.argv[1] if len(sys.argv) > 1 else "design.xml"
xml = build_xml(spec)
with open(out_path, "w") as f:
    f.write(xml)

a_act, h_act, k_act, n_act = spec.actuators()
print(f"Wrote {out_path}")
print(f"Ab/ad actuator: {a_act.name} (peak_torque={a_act.peak_torque:.2f} Nm)")
print(f"Hip actuator:   {h_act.name} (peak_torque={h_act.peak_torque:.2f} Nm)")
print(f"Knee actuator:  {k_act.name} (peak_torque={k_act.peak_torque:.2f} Nm)")
print(f"Ankle actuator: {n_act.name} (peak_torque={n_act.peak_torque:.2f} Nm)")
print(f"View it with: python view_joints.py {out_path}")
