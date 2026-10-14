"""UR3e with an OnRobot RG2 gripper - the showcase and outfeed arm.

The arm itself is ir_support's UR3e, straight from the course package; this file
only bolts the RG2 to its flange and keeps it there. The RG2 meshes are shared
with Arm A, so they are loaded from the Kawasaki folder rather than copied.

IMPORTANT - run from the same drive the packages are installed on. Swift strips
the drive letter out of every mesh path (see retrieveUrl in
swift/public/js/shapes.js) and then opens what is left on the drive Python is
running from, so a UR3e mesh inside a venv on another drive is never served and
the arm renders as nothing while still reporting [loaded]. SETUP.md covers it.

Run on its own:  python -m models.ur3e_rg2
"""

import os

from ir_support import UR3e
from spatialmath import SE3

try:
    from models.grippers import GripperedArm, RG2Gripper
except ImportError:                      # run from inside models/
    from grippers import GripperedArm, RG2Gripper

RG2_MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "Kawasaki_RS007N")


class UR3eRG2(GripperedArm, UR3e):
    """UR3e carrying an RG2. fkine() returns the point between the RG2 pads;
    flange_pose() returns the bare tool flange."""

    def __init__(self, base=None, gripper=True, mount=SE3()):
        super().__init__()
        if base is not None:
            self.base = base
        if gripper:
            self._attach_gripper(RG2Gripper(RG2_MESH_DIR, mount=mount))


if __name__ == "__main__":
    import swift
    import roboticstoolbox as rtb
    from math import pi

    robot = UR3eRG2(base=SE3(0, 0, 0))
    print(f"UR3e meshes: {robot.links_3d[0].filename}")
    env = swift.Swift()
    env.launch(realtime=True)
    robot.add_to_env(env)

    for q in rtb.jtraj(robot.q, [0, -pi / 3, pi / 3, -pi / 2, -pi / 2, 0], 60).q:
        robot.q = q
        env.step(0.02)
    robot.close_gripper(env, 0.020)      # close to a 20 mm gap
    robot.open_gripper(env)
    env.hold()
