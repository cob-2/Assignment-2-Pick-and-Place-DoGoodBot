"""Gripper models and the arm mixin that carries them.

Two hands are defined. Both expose the same interface, so any DHRobot3D arm can
hold either one:

    hand.shapes            list of Swift meshes to add to the environment
    hand.max_opening       widest jaw gap, metres
    hand.set_opening(w)    move the jaws to a gap of w metres
    hand.tcp               SE3, flange -> the point the jaws work at
    hand.update(T_flange)  redraw the hand at that flange pose

GripperedArm is a mixin for a DHRobot3D subclass. It keeps the hand glued to the
flange whenever q or base changes, adds the hand's meshes in add_to_env, and
sets self.tool so fkine() returns the gripper's working point instead of the
bare flange.

Usage:

    class MyArm(GripperedArm, DHRobot3D):
        def __init__(self):
            super().__init__(...)
            self._attach_gripper(DensoParallelHand(mesh_dir))
"""

import os

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry
from spatialmath import SE3


# ----------------------------------------------------------------------------
# DENSO COBOTTA PARALLEL HAND
# ----------------------------------------------------------------------------
# Denso's two-finger hand: both pads slide along the flange y axis, 0 to 15 mm
# each, mirrored, so the gap is 0 to 30 mm. The meshes are authored in the
# flange frame, so no mounting offset is needed.
class DensoParallelHand:
    STROKE = 0.015          # travel of one pad, metres
    FINGER_LENGTH = 0.060   # flange to fingertip plane, metres

    def __init__(self, mesh_dir, colour=(0.25, 0.27, 0.30, 1.0)):
        def mesh(name):
            return geometry.Mesh(os.path.join(mesh_dir, f"{name}.dae"), color=colour)

        self.base = mesh("gripper_base")
        self.left = mesh("left_finger")
        self.right = mesh("right_finger")
        self.shapes = [self.base, self.left, self.right]
        self.max_opening = 2 * self.STROKE
        self.half = self.STROKE
        # TCP at the fingertip plane, which matches tool_length in the
        # cobotta_cell files, so grasp depths carry over unchanged.
        self.tcp = SE3(0, 0, self.FINGER_LENGTH)
        self._flange = SE3()

    def set_opening(self, gap):
        self.half = float(np.clip(gap / 2.0, 0.0, self.STROKE))
        self.update(self._flange)

    @property
    def opening(self):
        return 2 * self.half

    def update(self, T_flange):
        self._flange = T_flange
        self.base.T = T_flange.A
        self.left.T = (T_flange * SE3(0, self.half, 0)).A
        self.right.T = (T_flange * SE3(0, -self.half, 0)).A


# ----------------------------------------------------------------------------
# ONROBOT RG2
# ----------------------------------------------------------------------------
# Four moving links per side driven by one angle. The pad gap and the fingertip
# height are not linear in that angle, so both are tabulated once at build time
# and inverted by interpolation. Same meshes as Arm A's hand.
class RG2Gripper:
    ANGLE_RANGE = (-0.558505, 0.785398)     # drive angle limits, radians

    def __init__(self, mesh_dir, mount=SE3(), samples=400):
        th = np.linspace(*self.ANGLE_RANGE, samples)
        gap, tip = [], []
        for t in th:
            P = self._link_poses(t)
            gap.append((P["R_finger"].A @ [0, 0.01895, 0, 1])[1]
                       - (P["L_finger"].A @ [0, 0.01895, 0, 1])[1])
            tip.append((P["L_finger"].A @ [0, 0, 0.052799, 1])[2])
        gap, tip = np.array(gap), np.array(tip)
        keep = gap >= 0
        self._lut_angle = th[keep][::-1]
        self._lut_gap = gap[keep][::-1]
        self._lut_tip = tip[keep][::-1]
        self.max_opening = float(gap.max())          # ~0.104 m

        def mesh(name, colour):
            return geometry.Mesh(os.path.join(mesh_dir, f"RG2_{name}.dae"), color=colour)

        grey, link, black = (0.88, 0.88, 0.90, 1), (0.75, 0.75, 0.78, 1), (0.12, 0.12, 0.12, 1)
        self.base = mesh("base_link", grey)
        self.parts = {"L_outer": mesh("outer_knuckle", link),
                      "R_outer": mesh("outer_knuckle", link),
                      "L_inner": mesh("inner_knuckle", link),
                      "R_inner": mesh("inner_knuckle", link),
                      "L_finger": mesh("inner_finger", black),
                      "R_finger": mesh("inner_finger", black)}
        self.shapes = [self.base] + list(self.parts.values())
        self.mount = mount                           # flange -> gripper base
        self._flange = SE3()
        self.tcp = SE3()
        self.set_opening(self.max_opening)

    @staticmethod
    def _link_poses(th):
        fin = SE3(0, -0.039592, 0.038177)
        L_ok = SE3(0, -0.017178, 0.125797) * SE3.Rx(-th)
        R_ok = SE3(0, 0.017178, 0.125797) * SE3.Rz(np.pi) * SE3.Rx(-th)
        return dict(L_outer=L_ok, R_outer=R_ok,
                    L_inner=SE3(0, -0.007678, 0.142297) * SE3.Rx(-th),
                    R_inner=SE3(0, 0.007678, 0.142297) * SE3.Rz(-np.pi) * SE3.Rx(-th),
                    L_finger=L_ok * fin * SE3.Rx(th),
                    R_finger=R_ok * fin * SE3.Rx(th))

    def set_opening(self, gap):
        self.opening = float(np.clip(gap, 0.0, self.max_opening))
        self._angle = float(np.interp(self.opening, self._lut_gap, self._lut_angle))
        tcp_z = float(np.interp(self.opening, self._lut_gap, self._lut_tip)) - 0.010
        self.tcp = self.mount * SE3(0, 0, tcp_z)     # point between the pads
        self.update(self._flange)

    def update(self, T_flange):
        self._flange = T_flange
        T = T_flange * self.mount
        self.base.T = T.A
        for name, P in self._link_poses(self._angle).items():
            self.parts[name].T = (T * P).A


# ----------------------------------------------------------------------------
# ARM MIXIN
# ----------------------------------------------------------------------------
class GripperedArm:
    """Keeps a hand attached to a DHRobot3D flange. List it before DHRobot3D."""

    def _attach_gripper(self, hand):
        self.gripper = hand
        self.tool = hand.tcp
        self._refresh_gripper()

    def __setattr__(self, name, value):
        # DHRobot3D redraws *before* storing the new value, which leaves the
        # gripper one assignment behind. Store first, then redraw.
        rtb.DHRobot.__setattr__(self, name, value)
        if name in ("q", "base") and hasattr(self, "_relation_matrices"):
            self._update_3dmodel()

    def _update_3dmodel(self):
        super()._update_3dmodel()
        self._refresh_gripper()

    def _refresh_gripper(self):
        hand = getattr(self, "gripper", None)
        if hand is not None:
            hand.update(self.flange_pose())

    def flange_pose(self, q=None):
        """Flange frame, i.e. fkine() without the tool transform."""
        q = self.q if q is None else q
        return SE3(self._get_transforms(q)[-1], check=False)

    def add_to_env(self, env):
        super().add_to_env(env)
        hand = getattr(self, "gripper", None)
        if hand is not None:
            for shape in hand.shapes:
                env.add(shape)

    def set_gripper(self, opening):
        self.gripper.set_opening(opening)
        self.tool = self.gripper.tcp
        self._refresh_gripper()

    def move_gripper(self, opening, env, steps=25, dt=0.02):
        for w in np.linspace(self.gripper.opening, opening, steps):
            self.set_gripper(w)
            env.step(dt)

    def open_gripper(self, env, steps=25):
        self.move_gripper(self.gripper.max_opening, env, steps)

    def close_gripper(self, env, width=0.0, steps=25):
        self.move_gripper(width, env, steps)
