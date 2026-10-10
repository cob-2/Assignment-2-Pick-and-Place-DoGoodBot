##  @file
#   @brief Kawasaki RS007N defined by standard DH parameters, with 3D meshes
#
#   Kinematics and meshes come from Kawasaki's official ROS package
#   (github.com/Kawasaki-Robotics/khi_robot, khi_rs_description, BSD licence).
#   The DH table below reproduces that URDF exactly, so a joint angle here is
#   the same as the joint angle on the real controller (JT1..JT6).
#
#   q = [0,0,0,0,0,0] is the robot standing straight up.

import os
import time
from math import pi, radians

import numpy as np
import roboticstoolbox as rtb
import spatialmath.base as spb
import swift
from spatialmath import SE3
from ir_support import DHRobot3D


class RS007N(DHRobot3D):

    def __init__(self, base=None):
        links = self._create_DH()

        # Mesh files sit next to this .py file (STL, metres)
        link3D_names = dict(link0="RS007N_J0",
                            link1="RS007N_J1",
                            link2="RS007N_J2",
                            link3="RS007N_J3",
                            link4="RS007N_J4",
                            link5="RS007N_J5",
                            link6="RS007N_J6",
                            color0=(0.55, 0.55, 0.55, 1),
                            color1=(0.95, 0.95, 0.95, 1),
                            color2=(0.95, 0.95, 0.95, 1),
                            color3=(0.95, 0.95, 0.95, 1),
                            color4=(0.95, 0.95, 0.95, 1),
                            color5=(0.95, 0.95, 0.95, 1),
                            color6=(0.30, 0.30, 0.30, 1))

        # Pose of each mesh (world frame) when q = qtest, taken from the URDF
        # joint origins + visual origins at the all-zero configuration.
        qtest = [0, 0, 0, 0, 0, 0]
        qtest_transforms = [
            spb.transl(0, 0, 0),                                   # base  J0
            spb.transl(0, 0, 0.36),                                # link1 J1
            spb.transl(0, 0, 0.36),                                # link2 J2
            spb.transl(0, 0, 0.715),                               # link3 J3
            spb.transl(0, 0, 1.09),                                # link4 J4
            spb.transl(0, 0, 1.09),                                # link5 J5
            spb.transl(0, 0, 1.168),                               # link6 J6 (flange)
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link3D_names, name="RS007N",
                         link3d_dir=current_path,
                         qtest=qtest, qtest_transforms=qtest_transforms)

        # Flange frame matches the URDF link6 frame
        self.tool = SE3.Rz(-pi / 2)
        if base is not None:
            self.base = base
        self.q = qtest

    # -------------------------------------------------------------------------
    def __setattr__(self, name, value):
        # DHRobot3D refreshes the meshes *before* storing the new q/base, which
        # leaves them one step behind. Store first, then refresh.
        rtb.DHRobot.__setattr__(self, name, value)
        if name in ("q", "base") and hasattr(self, "_relation_matrices"):
            self._update_3dmodel()

    # -------------------------------------------------------------------------
    def _create_DH(self):
        """Standard DH for the Kawasaki RS007N (lengths in metres)."""
        d      = [0.360,  0.0,   0.0,    0.375, 0.0,    0.078]
        a      = [0.0,    0.355, 0.0,    0.0,   0.0,    0.0]
        alpha  = [pi/2,   pi,    -pi/2,  pi/2,  -pi/2,  0.0]
        offset = [-pi/2,  pi/2,  -pi/2,  0.0,   0.0,    0.0]
        flip   = [True,   False, False,  False, False,  False]   # JT1 turns about -Z

        # Datasheet motion range (deg): JT1 ±180, JT2 ±135, JT3 ±155,
        #                               JT4 ±200, JT5 ±125, JT6 ±360
        lim = [180, 135, 155, 200, 125, 360]
        qlim = [[-radians(l), radians(l)] for l in lim]

        links = []
        for i in range(6):
            links.append(rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i],
                                        offset=offset[i], flip=flip[i],
                                        qlim=qlim[i]))
        return links

    # -------------------------------------------------------------------------
    def test(self):
        env = swift.Swift()
        env.launch(realtime=True)
        self.base = SE3(0.5, 0.5, 0)
        self.add_to_env(env)

        q_goal = [0, pi/4, pi/2, 0, pi/4, 0]
        for q in rtb.jtraj(self.q, q_goal, 60).q:
            self.q = q
            env.step(0.02)
        time.sleep(3)


if __name__ == "__main__":
    r = RS007N()
    print(r)
    r.test()
