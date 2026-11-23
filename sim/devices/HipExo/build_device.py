"""Build the HipExo_L1 MyoAssist device from the validated bench model (exo_mujoco/build_exo_bench.py).

Usage:  python build_device.py PLANT_PARAMS.npz [--arm-length 0.24]
Writes L1model.xml next to this file and prints the joint_dynamics_add block for the training config.

What comes from the bench fit, per motor (id1 = left, id2 = right):
    arm mass        m = A / (g r), centre of mass r down the thigh in the motor's swing plane
    damping  Fv     -> added to the human hip_flexion joint
    friction Fc     -> added to the human hip_flexion joint (frictionloss)
    armature        J - m r^2 - i_com + drive inertia, the same formula as the bench model -> added to the joint
The arm mass sits on the thigh body here, so its m r^2 comes from the body and only the rest goes to the armature,
exactly as in the bench model. Damping, friction and armature act between pelvis and thigh, which in the human model
is the hip_flexion joint; the device format cannot set those, so the env adds them (joint_dynamics_add).

Geometry is the bench model's display geometry (C-shaped hip shell, links that bend inward by LINK_INSET, oval cuff
CUFF_DIST down the link), placed on the myolegs22 hip centres. Not identified by the bench and so still estimates:
the arm length r (assumed, as in the bench model) and the hip-side mass (frame and motors; the bench frame is fixed).

Frames: hipexo_frame attaches to the pelvis (x forward, y up, z right). hipexo_thigh_r/l attach to femur_r/l, whose
origin is the hip centre and whose axes match the pelvis frame at zero hip angle.
"""

import argparse
import os

import numpy as np

G = 9.81
HIP = np.array([-0.0707, -0.0661, 0.0835])  # myolegs22 right hip centre in the pelvis frame (left: z -> -z)
CUFF_DIST = 0.25  # cuff centre down the link from the motor shaft (m), as in build_exo_bench.py
LINK_INSET = 0.03  # inward bend of the link from the motor to the cuff (m)
CUFF_R_SIDE, CUFF_R_FRONT = 0.055, 0.065
SHAFT_OUT = LINK_INSET + CUFF_R_SIDE  # motor shaft plane outboard of the thigh axis, so the cuff wraps the thigh
HOUSING_R, HOUSING_LEN = 0.0345, 0.03  # AKE60-8: 69 mm across; shell-to-arm gap as in the bench model
# Hip side, not identified by the bench (fixed frame). Estimates from sim/models/hip_exo.xml.
FRAME_MASS, MOTOR_MASS = 0.8, 0.6

p = argparse.ArgumentParser()
p.add_argument("params")
p.add_argument("--arm-length", type=float, default=0.24, help="hinge to arm centre of mass (m); assumed, as in the bench")
args = p.parse_args()
P = np.load(args.params)
r = args.arm_length


def box(pos, xaxis, yaxis, size, rgba, name=None):
    x = np.asarray(xaxis, float) / np.linalg.norm(xaxis)
    y = np.asarray(yaxis, float) - np.dot(yaxis, x) * x
    y /= np.linalg.norm(y)
    nm = f'name="{name}" ' if name else ""
    return (f'<geom {nm}type="box" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}" '
            f'xyaxes="{x[0]:.4f} {x[1]:.4f} {x[2]:.4f} {y[0]:.4f} {y[1]:.4f} {y[2]:.4f}" mass="0" contype="0" conaffinity="0" rgba="{rgba}"/>')


def strip(pts, width_dir, half_w, half_t, rgba):
    out = []
    for p0, p1 in zip(pts[:-1], pts[1:]):
        seg = p1 - p0
        out.append(box((p0 + p1) / 2, seg, width_dir, (np.linalg.norm(seg) / 2 + 0.002, half_w, half_t), rgba))
    return out


def hip_shell(n=14, a=0.13, height=0.10, thick=0.008):
    """C-shaped band behind the pelvis, open at the front, at motor height. Display only."""
    b = HIP[2] + SHAFT_OUT - HOUSING_LEN
    phi = np.linspace(-np.pi / 2, np.pi / 2, n + 1)
    pts = [np.array([HIP[0] - a * np.cos(f), HIP[1], b * np.sin(f)]) for f in phi]
    geoms = []
    for p0, p1 in zip(pts[:-1], pts[1:]):
        seg = p1 - p0
        geoms.append(box((p0 + p1) / 2, seg, (0, 1, 0), (np.linalg.norm(seg) / 2 + 0.002, height / 2, thick / 2), "0.6 0.6 0.6 1"))
    return geoms


def thigh(side, mass, n=12):
    """Thigh body in the femur frame: fitted arm mass, bent link, oval cuff."""
    s_ = "r" if side > 0 else "l"
    u, v, out = np.array([0, -1.0, 0]), np.array([1.0, 0, 0]), np.array([0, 0, side])
    shaft = (HIP[2] * 0 + SHAFT_OUT) * out  # femur origin is the hip centre; shaft plane is SHAFT_OUT outboard of it
    s = np.linspace(0, CUFF_DIST + 0.05, n + 1)
    bend = LINK_INSET * (1 - np.cos(np.pi * np.clip(s / CUFF_DIST, 0, 1))) / 2
    link = strip([shaft + si * u - bi * out for si, bi in zip(s, bend)], v, 0.015, 0.0025, "0.55 0.42 0.25 1")
    c = shaft + CUFF_DIST * u - SHAFT_OUT * out  # = on the thigh axis
    phi = np.linspace(0, 2 * np.pi, 25)
    ring = [c + CUFF_R_SIDE * np.cos(f) * out + CUFF_R_FRONT * np.sin(f) * v for f in phi]
    cuff = []
    for p0, p1 in zip(ring[:-1], ring[1:]):
        seg = p1 - p0
        cuff.append(box((p0 + p1) / 2, seg, u, (np.linalg.norm(seg) / 2 + 0.002, 0.04, 0.004), "0.75 0.6 0.4 1"))
    com = shaft + r * u
    body = [f'<body name="hipexo_thigh_{s_}" pos="0 0 0">',
            f'    <inertial pos="{com[0]:.5f} {com[1]:.5f} {com[2]:.5f}" mass="{mass:.5f}" diaginertia="1e-05 1e-05 1e-05"/>']
    body += ["    " + g for g in link + cuff]
    body.append(f'    <site name="hipexo_cuff_site_{s_}" pos="{c[0]:.4f} {c[1]:.4f} {c[2] + side * CUFF_R_SIDE:.4f}" size="0.01"/>')
    body.append("</body>")
    return body


thighs, dyn = [], {}
for m_id, side in (("1", -1), ("2", +1)):  # id1 = left, id2 = right
    J, Fc, Fv, Gc, Gs = P[f"plant_id{m_id}"]
    mass = np.hypot(Gc, Gs) / (G * r)
    armature = J - mass * r**2 - 1e-5
    if armature < 0:
        raise SystemExit(f"id{m_id}: arm length {r} m gives m r^2 > fitted J; choose a shorter arm")
    armature += float(P[f"drive_inertia_id{m_id}"]) if f"drive_inertia_id{m_id}" in P.files else 0.0
    joint = f"hip_flexion_{'r' if side > 0 else 'l'}"
    dyn[joint] = {"damping": round(float(Fv), 5), "frictionloss": round(float(Fc), 5), "armature": round(float(armature), 5)}
    thighs += thigh(side, mass)
    print(f"id{m_id} -> {joint}: arm {mass:.4f} kg at {r} m, damping {Fv:.5f}, frictionloss {Fc:.5f}, armature {armature:.5f}")

shaft_z = HIP[2] + SHAFT_OUT
frame = [f'<body name="hipexo_frame" pos="0 0 0">',
         f'    <inertial pos="{HIP[0] - 0.12:.4f} {HIP[1]:.4f} 0" mass="{FRAME_MASS}" diaginertia="0.005 0.005 0.005"/>']
frame += ["    " + g for g in hip_shell()]
for side, s_ in ((+1, "r"), (-1, "l")):
    z0, z1 = side * (shaft_z - HOUSING_LEN), side * shaft_z
    frame += [f'    <body name="hipexo_module_{s_}" pos="{HIP[0]:.4f} {HIP[1]:.4f} {(z0 + z1) / 2:.4f}">',
              f'        <inertial pos="0 0 0" mass="{MOTOR_MASS}" diaginertia="0.0004 0.0004 0.0004"/>',
              f'        <geom type="cylinder" size="{HOUSING_R} {HOUSING_LEN / 2}" mass="0" contype="0" conaffinity="0" rgba="0.15 0.15 0.15 1"/>',
              "    </body>"]
frame.append("</body>")

ind = "\n        "
xml = f"""<!-- Generated by build_device.py from {os.path.basename(args.params)}; arm length {r} m is assumed, not measured.
     Hip-side masses ({FRAME_MASS} kg frame, {MOTOR_MASS} kg per motor) are estimates, not from the bench fit.
     Motor damping, friction and armature are added to the human hip joints by the env (joint_dynamics_add). -->
<mujoco model="HipExoL1">
    <compiler angle="radian"/>
    <worldbody>
        {ind.join(frame)}
        {ind.join(thighs)}
    </worldbody>
</mujoco>
"""
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "L1model.xml")
open(out, "w").write(xml)
print("wrote", out)
print('\nAdd to env_params in the training config:\n"joint_dynamics_add": ' + str(dyn).replace("'", '"'))
