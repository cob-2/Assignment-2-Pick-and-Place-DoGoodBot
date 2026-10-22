import numpy as np
import spatialgeometry as geometry
from spatialmath import SE3

import scene_config as cfg
import cobotta_collision_detection as cell
import kawasaki_collision_detection as kcd
import ur3e_collision_detection as ucd


OBSTACLE = dict(
    size=(0.10, 0.10, 0.06),
    colour=(0.90, 0.10, 0.10, 1.0),
    xy=(cfg.TRANSFER["x"], cfg.TRANSFER["y"]),   # over the hand-off holder, where the Kawasaki and COBOTTA meet
    above=0.20,             # gap between the top of a card in the holder and the obstacle's underside (m)
    margin=0.03,            # an arm part closer than this to the obstacle counts as reaching it (m)
    lookahead=10,           # motion steps the current joint velocity is projected ahead
    spacing=0.015,          # point spacing on the obstacle's faces (m)
    parked=SE3(0, 0, -100),
)


class ObstacleGuard:
    def __init__(self, task):
        self.task = task
        self.present = False
        self.shape = None
        self.last = {}
        o = OBSTACLE
        top = cfg.BENCH_H + cfg.RACK["base_h"] + cfg.SLAB["h"]
        self.pose = SE3(o["xy"][0], o["xy"][1], top + o["above"] + o["size"][2] / 2)
        self.points = cell.surface_points(o["size"], o["spacing"]) + self.pose.t

    def build(self, env):
        self.shape = geometry.Cuboid(scale=list(OBSTACLE["size"]), pose=OBSTACLE["parked"],
                                     color=OBSTACLE["colour"])
        env.add(self.shape)

    def drop(self):
        self.present = True
        self.last.clear()
        if self.shape is not None:
            self.shape.T = self.pose.A

    def remove(self):
        self.present = False
        if self.shape is not None:
            self.shape.T = OBSTACLE["parked"].A

    def arms(self):
        t = self.task
        out = [("COBOTTA", t.robot, lambda r, q: cell.arm_ellipsoids(r, t.cfg, q))]
        if t.vault is not None:
            out.append(("Kawasaki", t.vault.robot, lambda r, q: kcd.arm_ellipsoids(r, q)))
        if t.outfeed is not None:
            out.append(("UR3e", t.outfeed.robot, lambda r, q: ucd.arm_ellipsoids(r, q)))
        return out

    def touching(self, fn, robot, q):
        """The first arm part whose ellipsoid, grown by the margin, holds an obstacle point."""
        for item in fn(robot, q):
            centre, R, radii = item[-3:]
            rel = (self.points - np.asarray(centre)) @ np.asarray(R)
            grown = np.asarray(radii, dtype=float) + OBSTACLE["margin"]
            if np.any(np.sum((rel / grown) ** 2, axis=1) < 1.0):
                return item[1] if isinstance(item[1], str) else f"link{item[0]}"
        return None

    def check(self):
        """(arm, part, when) for the first arm at the obstacle now ('now') or on course
        to reach it within `lookahead` steps of its current joint motion ('ahead');
        None when the obstacle is out or nothing is near it."""
        if not self.present:
            return None
        for name, robot, fn in self.arms():
            q = np.asarray(robot.q, dtype=float)
            prev = self.last.get(name, q)
            self.last[name] = q.copy()
            lim = np.asarray(robot.qlim, dtype=float)
            ahead = np.clip(q + OBSTACLE["lookahead"] * (q - prev), lim[0], lim[1])
            for when, qq in (("now", q), ("ahead", ahead)):
                part = self.touching(fn, robot, qq)
                if part is not None:
                    return name, part, when
        return None

    def settle(self):
        """Forget the last poses, so a stopped arm isn't treated as still moving."""
        self.last.clear()
