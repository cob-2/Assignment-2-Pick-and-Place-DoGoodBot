import os
import time
from math import pi, radians


import numpy as np
import roboticstoolbox as rtb
import spatialmath.base as spb
import swift
from spatialmath import SE3
from ir_support import DHRobot3D
import spatialgeometry as geometry 


class RS007N(DHRobot3D):

    def __init__(self, base=None):
        links = self._create_DH()
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
        self._build_gripper(current_path)
        if base is not None:
            self.base = base
        self.q = qtest


    def __setattr__(self, name, value):
        rtb.DHRobot.__setattr__(self, name, value)
        if name in ("q", "base") and hasattr(self, "_relation_matrices"):
            self._update_3dmodel()

    def _create_DH(self):
        d      = [0.360,  0.0,   0.0,    0.375, 0.0,    0.078]
        a      = [0.0,    0.355, 0.0,    0.0,   0.0,    0.0]
        alpha  = [pi/2,   pi,    -pi/2,  pi/2,  -pi/2,  0.0]
        offset = [-pi/2,  pi/2,  -pi/2,  0.0,   0.0,    0.0]
        flip   = [False,  False, False,  False, False,  False]
        # No 'flip': roboticstoolbox 1.3.x handles flip=True wrongly (its IK and fkine
        # disagree). Without it the arm reaches exactly the same poses, but the model's
        # joint 1 angle has the opposite sign to the real controller's JT1 (q1 = -JT1).
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
    
    #  OnRobot RG2 gripper 
    def _update_3dmodel(self):
        super()._update_3dmodel()
        if getattr(self, "gripper_shapes", None):
            self._update_gripper()

    def add_to_env(self, env):
        super().add_to_env(env)
        for shape in self.gripper_shapes:
            env.add(shape)

    def flange_pose(self, q=None):
        q = self.q if q is None else q
        return SE3(self._get_transforms(q)[-1], check=False) * SE3.Rz(-pi / 2)

    @staticmethod
    def _rg2_link_poses(th):
        fin = SE3(0, -0.039592, 0.038177)
        L_ok = SE3(0, -0.017178, 0.125797) * SE3.Rx(-th)
        R_ok = SE3(0, 0.017178, 0.125797) * SE3.Rz(pi) * SE3.Rx(-th)
        return dict(L_outer=L_ok, R_outer=R_ok,
                    L_inner=SE3(0, -0.007678, 0.142297) * SE3.Rx(-th),
                    R_inner=SE3(0, 0.007678, 0.142297) * SE3.Rz(-pi) * SE3.Rx(-th),
                    L_finger=L_ok * fin * SE3.Rx(th),
                    R_finger=R_ok * fin * SE3.Rx(th))

    def _build_gripper(self, mesh_dir):
        th = np.linspace(-0.558505, 0.785398, 400)
        gap, tip = [], []
        for t in th:
            P = self._rg2_link_poses(t)
            gap.append((P["R_finger"].A @ [0, 0.01895, 0, 1])[1] - (P["L_finger"].A @ [0, 0.01895, 0, 1])[1])
            tip.append((P["L_finger"].A @ [0, 0, 0.052799, 1])[2])
        gap, tip = np.array(gap), np.array(tip)
        keep = gap >= 0
        self._lut_angle, self._lut_gap, self._lut_tip = th[keep][::-1], gap[keep][::-1], tip[keep][::-1]
        self.MAX_OPENING = float(gap.max())                       # ~0.104 m

        mesh = lambda n, c: geometry.Mesh(os.path.join(mesh_dir, f"RG2_{n}.dae"), color=c)
        grey, link, black = (0.88, 0.88, 0.90, 1), (0.75, 0.75, 0.78, 1), (0.12, 0.12, 0.12, 1)
        self._g_base = mesh("base_link", grey)
        self._g_parts = {"L_outer": mesh("outer_knuckle", link), "R_outer": mesh("outer_knuckle", link),
                         "L_inner": mesh("inner_knuckle", link), "R_inner": mesh("inner_knuckle", link),
                         "L_finger": mesh("inner_finger", black), "R_finger": mesh("inner_finger", black)}
        self.gripper_shapes = [self._g_base] + list(self._g_parts.values())
        self.set_gripper(self.MAX_OPENING)

    def _update_gripper(self):
        F = self.flange_pose()
        self._g_base.T = F.A
        for name, P in self._rg2_link_poses(self._rg2_angle).items():
            self._g_parts[name].T = (F * P).A

    def set_gripper(self, opening):
        self.gripper_opening = float(np.clip(opening, 0.0, self.MAX_OPENING))
        self._rg2_angle = float(np.interp(self.gripper_opening, self._lut_gap, self._lut_angle))
        tcp_z = float(np.interp(self.gripper_opening, self._lut_gap, self._lut_tip)) - 0.010
        self.tool = SE3.Rz(-pi / 2) * SE3(0, 0, tcp_z)    # fkine() = point between the pads
        self._update_gripper()

    def move_gripper(self, opening, env, steps=25, dt=0.02):
        for w in np.linspace(self.gripper_opening, opening, steps):
            self.set_gripper(w)
            env.step(dt)

    def open_gripper(self, env, steps=25):
        self.move_gripper(self.MAX_OPENING, env, steps)

    def close_gripper(self, env, width=0.0, steps=25):
        self.move_gripper(width, env, steps)

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