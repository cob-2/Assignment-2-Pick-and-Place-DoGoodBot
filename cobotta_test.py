"""COBOTTA test rig - DH check, parallel gripper, and a Swift slider per joint.

Run:  python cobotta_test.py
"""

import os
from math import pi

import numpy as np
import swift
import spatialgeometry as geometry
from spatialmath import SE3
from roboticstoolbox import RevoluteDH, DHRobot

try:
    from ir_support.robots.DHRobot3D import DHRobot3D
except ImportError:
    from ir_support import DHRobot3D


# ----------------------------------------------------------------------------
# DH TABLE
# ----------------------------------------------------------------------------
# d (m), a (m), alpha, theta offset, joint limits (deg)
DH_TABLE = [
    (0.1800,  0.0000, -pi / 2, 0.0,    (-150.0, 150.0)),
    (0.0000, -0.1650,  0.0,    pi / 2, (-60.0,  100.0)),
    (0.0200,  0.0120, -pi / 2, pi / 2, (18.0,   140.0)),
    (0.1775,  0.0000,  pi / 2, 0.0,    (-170.0, 170.0)),
    (-0.0645, 0.0000,  pi / 2, pi,     (-95.0,  135.0)),
    (0.0420,  0.0000,  0.0,    0.0,    (-170.0, 170.0)),
]

QLIM_DEG = [lim for *_, lim in DH_TABLE]


def cobotta_links():
    return [RevoluteDH(d=d, a=a, alpha=al, offset=of,
                       qlim=[np.deg2rad(lo), np.deg2rad(hi)])
            for d, a, al, of, (lo, hi) in DH_TABLE]


def COBOTTA_DH():
    return DHRobot(cobotta_links(), name="COBOTTA")


# ----------------------------------------------------------------------------
# MESHES
# ----------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(HERE, "models", "cobotta_meshes")

LINK3D_NAMES = dict(link0="base_link", link1="J1", link2="J2", link3="J3",
                    link4="J4", link5="J5", link6="J6")

QTEST = [0, 0, 0, 0, 0, 0]
QTEST_TRANSFORMS = [
    SE3(0, 0, 0),
    SE3(0, 0, 0),
    SE3(0, 0, 0.1800),
    SE3(0, 0, 0.3450),
    SE3(-0.012, 0.020, 0.4330),
    SE3(-0.012, 0.000, 0.5225),
    SE3(-0.012, -0.0445, 0.5645),
]


# ----------------------------------------------------------------------------
# ROBOT
# ----------------------------------------------------------------------------
class COBOTTA(DHRobot3D):
    def __init__(self, base=SE3()):
        super().__init__(
            cobotta_links(),
            link3D_names=LINK3D_NAMES,
            link3d_dir=MESH_DIR,
            name="COBOTTA",
            qtest=QTEST,
            qtest_transforms=[T.A for T in QTEST_TRANSFORMS],
        )
        self.base = base
        self.q = np.array(QTEST, dtype=float)


# ----------------------------------------------------------------------------
# PARALLEL GRIPPER
# ----------------------------------------------------------------------------
# Denso's parallel hand: both fingers slide along the flange y axis,
# 0 to 15 mm each, mirrored. Meshes sit in the flange frame.
GRIPPER_STROKE = 0.015


class ParallelGripper:
    def __init__(self, colour=(0.25, 0.27, 0.30, 1.0)):
        self.base = geometry.Mesh(os.path.join(MESH_DIR, "gripper_base.dae"), color=colour)
        self.left = geometry.Mesh(os.path.join(MESH_DIR, "left_finger.dae"), color=colour)
        self.right = geometry.Mesh(os.path.join(MESH_DIR, "right_finger.dae"), color=colour)
        self.opening = GRIPPER_STROKE

    def add_to_env(self, env):
        for m in (self.base, self.left, self.right):
            env.add(m)

    def update(self, T_flange, opening=None):
        if opening is not None:
            self.opening = float(np.clip(opening, 0.0, GRIPPER_STROKE))
        self.base.T = T_flange.A
        self.left.T = (T_flange * SE3(0, self.opening, 0)).A
        self.right.T = (T_flange * SE3(0, -self.opening, 0)).A


# ----------------------------------------------------------------------------
# DH VERIFICATION AGAINST THE URDF
# ----------------------------------------------------------------------------
JOINT_ORIGINS = [(0, 0, 0), (0, 0, 0.180), (0, 0, 0.165),
                 (-0.012, 0.020, 0.088), (0, -0.020, 0.0895), (0, -0.0445, 0.042)]
JOINT_AXES = ["z", "y", "y", "z", "y", "z"]


def urdf_fk(q):
    T = SE3()
    for i in range(6):
        T = T * SE3(*JOINT_ORIGINS[i])
        T = T * (SE3.Rz(q[i]) if JOINT_AXES[i] == "z" else SE3.Ry(q[i]))
    return T


def verify_dh(samples=200):
    robot = COBOTTA_DH()
    rng = np.random.default_rng(0)
    worst = max(float(np.abs(urdf_fk(q).A - robot.fkine(q).A).max())
                for q in rng.uniform(-1.5, 1.5, size=(samples, 6)))
    print(f"DH vs URDF, worst mismatch over {samples} configurations: {worst:.2e} "
          f"({'PASS' if worst < 1e-9 else 'FAIL'})")
    return worst


# ----------------------------------------------------------------------------
# SWIFT TEACH PENDANT
# ----------------------------------------------------------------------------
def main():
    verify_dh()

    robot = COBOTTA()
    gripper = ParallelGripper()

    env = swift.Swift()
    env.launch(realtime=True)
    robot.add_to_env(env)
    gripper.add_to_env(env)

    state = dict(q=np.array(QTEST, dtype=float), grip=GRIPPER_STROKE, dirty=True)

    def joint_cb(index):
        def cb(value):
            state["q"][index] = np.deg2rad(value)
            state["dirty"] = True
        return cb

    def grip_cb(value):
        state["grip"] = value / 1000.0
        state["dirty"] = True

    readout = swift.Label("")
    env.add(readout)

    for i, (lo, hi) in enumerate(QLIM_DEG):
        env.add(swift.Slider(joint_cb(i), min=lo, max=hi, step=1,
                             value=float(np.rad2deg(state["q"][i])),
                             desc=f"q{i + 1}", unit="&#176;"))

    env.add(swift.Slider(grip_cb, min=0, max=GRIPPER_STROKE * 1000, step=0.5,
                         value=GRIPPER_STROKE * 1000, desc="gripper", unit="mm"))

    def refresh():
        robot.q = state["q"]
        robot._update_3dmodel()
        T = robot.fkine(state["q"])
        gripper.update(T, state["grip"])
        p = T.t
        rpy = np.rad2deg(T.rpy())
        readout.desc = (f"TCP  x {p[0]:+.3f}  y {p[1]:+.3f}  z {p[2]:+.3f} m   |   "
                        f"rpy {rpy[0]:+.1f} {rpy[1]:+.1f} {rpy[2]:+.1f} deg   |   "
                        f"grip {state['grip'] * 1000:.1f} mm")

    refresh()
    while True:
        if state["dirty"]:
            refresh()
            state["dirty"] = False
        env.step(0.05)


if __name__ == "__main__":
    main()
