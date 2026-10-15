"""Arm A (Kawasaki RS007N + RG2) vault task: fetch a card from the stands for the
COBOTTA, and put a rejected card back in its slot.

    vault = VaultArm(env, robot, shapes, gate)        # robot = the Kawasaki from the scene
    vault.fetch("card_c1r2")      # stand -> hand-off holder
    vault.put_back("card_c1r2")   # Return mount -> its original slot

Every move is checked with kawasaki_collision_detection (scene point cloud vs the
arm's fitted ellipsoids). The carried card is checked as a box, and the COBOTTA's
own link ellipsoids are added to the Kawasaki's world, so the two arms can't
run into each other at the shared hand-off holder.
Moves into and out of a groove are straight lines; the longer moves are joint
paths, re-planned through a via pose if the direct path is blocked.
"""

import numpy as np
from math import pi
from spatialmath import SE3

import scene_config as cfg
import kawasaki_collision_detection as kc


ARM = "A_vault"
TOOL_DOWN = SE3.Rx(pi)          # gripper pointing down, fingers closing across the slab (world y)

TASK = dict(
    slot_lift=0.20,             # lift above a stand: the slab clears the slabs on the other stands
    low_lifts=(0.14, 0.12, 0.10, 0.08),   # holder / mount: highest of these the arm can reach
    approach_open=0.030,        # finger gap while approaching a slab (m)
    grip_depth=0.030,           # fingers close this far below the slab's top edge
    steps=40,                   # joint path steps
    straight_steps=15,          # steps for a straight in/out move
    dt=0.03,
    gripper_steps=12,
    replan_tries=12,
)


def slab_size():
    return (cfg.SLAB["w"], cfg.SLAB["t"], cfg.SLAB["h"])


def holder_rest():
    return SE3(cfg.TRANSFER["x"], cfg.TRANSFER["y"],
               cfg.BENCH_H + cfg.RACK["base_h"] + cfg.SLAB["h"] / 2)


def return_rest():
    return SE3(*cfg.mount_slab_xyz("return"))


def slot_rest(col, row):
    return SE3(*cfg.slot_xyz(col, row))


def grasp_from_rest(rest):
    """Grasp pose for a slab standing at 'rest': fingers grip_depth below its top edge."""
    return SE3(rest.t[0], rest.t[1], rest.t[2] + cfg.SLAB["h"] / 2 - TASK["grip_depth"]) * TOOL_DOWN


# ============================================================================
# THE COBOTTA AS AN OBSTACLE
# ============================================================================
class ArmCloud:
    """Points on the COBOTTA's link ellipsoids, refreshed from its current joint angles.
    Looks like a kawasaki_collision_detection.Obstacle to the checker."""

    def __init__(self, name, robot, checker_cfg, points_per=160):
        import cobotta_collision_detection as cell
        self._cell = cell
        self.name = name
        self.robot = robot
        self.cfg = checker_cfg
        self.active = True
        self.ignore_links = set()
        k = np.arange(points_per) + 0.5                       # even points on a unit sphere
        phi = np.arccos(1 - 2 * k / points_per)
        th = pi * (1 + 5 ** 0.5) * k
        self._unit = np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
        self._key = None

    def points(self):
        key = np.asarray(self.robot.q, dtype=float).tobytes() + str(getattr(self.robot, "gripper_opening", 0)).encode()
        if key != self._key:
            pts = []
            for _, centre, R, radii in self._cell.arm_ellipsoids(self.robot, self.cfg, self.robot.q):
                pts.append(centre + (self._unit * radii) @ R.T)
            self._pts = np.vstack(pts)
            self._lo, self._hi = self._pts.min(0), self._pts.max(0)
            self._key = key
        return self._pts

    def near(self, centre, radius):
        self.points()
        gap = np.maximum(0.0, np.maximum(self._lo - centre, centre - self._hi))
        return float(np.linalg.norm(gap)) < radius


# ============================================================================
# THE KAWASAKI
# ============================================================================
class VaultArm:
    def __init__(self, env, robot, shapes, gate=None, cobotta=None, cobotta_cfg=None, say=print):
        self.env, self.robot, self.shapes = env, robot, shapes
        self.gate = gate if gate is not None else env
        self.say = say
        self.home = np.array(cfg.ROBOTS[ARM]["q0"], dtype=float)
        self.world = kc.world_from_scene(cfg)
        if cobotta is not None and cobotta_cfg is not None:
            self.world.obstacles.append(ArmCloud("cobotta", cobotta, cobotta_cfg))
        self.carried = None                   # (shape name, T_tcp_shape)
        self.replans = 0
        self.faults = 0
        self.slot_of = {f"card_c{c}r{r}": (c, r)
                        for r in range(cfg.SLOTS["rows"]) for c in range(cfg.SLOTS["cols"])}
        self.in_stand = {n for n in self.slot_of if n in shapes}
        self._seeds = [self.home, [0, 0.5, 1.6, 0, 1.0, 0], [0.4, 0.3, 1.4, 0, 1.2, 0],
                       [-0.4, 0.3, 1.4, 0, 1.2, 0], [pi, 0.5, 1.6, 0, 1.0, 0]]
        self.robot.q = self.home
        self.robot.set_gripper(TASK["approach_open"])

    # ---------------------------------------------------------------- state
    def available(self):
        """Cards still standing in the vault, in slot order."""
        return [n for n in self.slot_of if n in self.in_stand]

    def sync(self):
        """Copy every object's current pose from Swift into the Kawasaki's world
        (the COBOTTA moves cards, lids and boxes around)."""
        held = self.carried[0] if self.carried else None
        for ob in self.world.obstacles:
            shape = self.shapes.get(ob.name)
            if shape is not None and ob.name != held:
                ob.pose = SE3(np.array(shape.T), check=False)

    def reset(self):
        self.carried = None
        self.in_stand = {n for n in self.slot_of if n in self.shapes}
        self.robot.set_gripper(TASK["approach_open"])
        self.robot.q = self.home
        for ob in self.world.obstacles:
            ob.active = True
        self.sync()

    # ---------------------------------------------------------------- motion
    def _carry(self):
        return [(slab_size(), self.carried[1])] if self.carried else ()

    def _follow(self, qs):
        for q in qs:
            self.robot.q = q
            if self.carried:
                name, grip = self.carried
                self.shapes[name].T = (self.robot.fkine(q) * grip).A
            self.gate.step(TASK["dt"])

    def _ik(self, T):
        return kc.solve_ik(self.robot, T, [self.robot.q] + self._seeds)

    def plan(self, q_goal):
        """Joint path to q_goal: direct if clear, otherwise through a via pose."""
        qs, why = kc.safe_jtraj(self.robot, self.world, q_goal, steps=TASK["steps"], carried=self._carry())
        if qs is not None:
            return qs
        self.say(f"    Kawasaki: path blocked ({why}) - re-planning")
        q0 = np.array(self.robot.q)
        lifted = lambda q: np.array([q[0], q[1] - 0.35, q[2] - 0.25, q[3], q[4], q[5]])
        candidates = [self.home, lifted(q0), lifted(q_goal)]
        rng = np.random.default_rng(7)
        lim = np.asarray(self.robot.qlim)
        for k in range(TASK["replan_tries"]):
            q_mid = candidates[k] if k < len(candidates) else rng.uniform(lim[0], lim[1])
            q_mid = np.clip(q_mid, lim[0], lim[1])
            leg1, _ = kc.safe_jtraj(self.robot, self.world, q_mid, q_start=q0, steps=TASK["steps"], carried=self._carry())
            if leg1 is None:
                continue
            leg2, _ = kc.safe_jtraj(self.robot, self.world, q_goal, q_start=q_mid, steps=TASK["steps"], carried=self._carry())
            if leg2 is not None:
                self.replans += 1
                return np.vstack((leg1, leg2))
        self.faults += 1
        return None

    def move_to(self, q_goal):
        qs = self.plan(np.asarray(q_goal, dtype=float))
        if qs is None:
            self.say("    Kawasaki: no safe path - holding")
            return False
        self._follow(qs)
        return True

    def move_pose(self, T):
        q = self._ik(T)
        if q is None:
            self.faults += 1
            self.say("    Kawasaki: pose out of reach")
            return False
        return self.move_to(q)

    def straight(self, T, ignore=()):
        """Straight-line TCP move (into / out of a groove), collision-checked."""
        with self.world.ignoring(list(ignore)):
            qs = kc.straight_path(self.robot, T, steps=TASK["straight_steps"], seeds=self._seeds)
            if qs is None:
                self.faults += 1
                self.say("    Kawasaki: no straight-line path")
                return False
            k, hit = kc.path_collision(self.robot, self.world, qs, carried=self._carry(), check_every=1)
            if hit:
                self.faults += 1
                self.say(f"    Kawasaki: straight move blocked ({hit})")
                return False
        self._follow(qs)
        return True

    def gripper_to(self, opening):
        for w in np.linspace(self.robot.gripper_opening, opening, TASK["gripper_steps"]):
            self.robot.set_gripper(w)
            self.gate.step(TASK["dt"])

    def _low_lift(self, grasp):
        """Highest approach over the holder / mount the arm can still reach straight down from."""
        for lift in TASK["low_lifts"]:
            if self._ik(SE3(0, 0, lift) * grasp) is not None:
                return lift
        return TASK["low_lifts"][-1]

    # ---------------------------------------------------------------- pick / place
    def pick(self, name, rest, lift, fixtures=()):
        """Take the slab 'name' standing at 'rest'. fixtures = the stand / holder pieces round it."""
        self.sync()
        g = grasp_from_rest(rest)
        a = SE3(0, 0, lift) * g
        self.robot.set_gripper(TASK["approach_open"])
        ok = self.move_pose(a)
        ok = ok and self.straight(g, ignore=[name, *fixtures])
        if not ok:
            return False
        self.gripper_to(cfg.SLAB["t"])
        T = self.robot.fkine(self.robot.q)
        self.carried = (name, T.inv() * SE3(np.array(self.shapes[name].T), check=False))
        self.world.get(name).active = False
        return self.straight(a, ignore=fixtures)

    def place(self, rest, lift, fixtures=()):
        """Stand the carried slab at 'rest' and let go."""
        g = grasp_from_rest(rest)
        a = SE3(0, 0, lift) * g
        ok = self.move_pose(a) and self.straight(g, ignore=fixtures)
        if not ok:
            return False
        name, _ = self.carried
        self.shapes[name].T = rest.A                     # settle exactly in the slot
        ob = self.world.get(name)
        ob.pose, ob.active = rest, True
        self.carried = None
        self.gripper_to(TASK["approach_open"])
        return self.straight(a, ignore=[name, *fixtures])

    # ---------------------------------------------------------------- the two jobs
    def fetch(self, name):
        """Stand -> hand-off holder, then back to the ready pose."""
        if name not in self.in_stand:
            self.say(f"   {name} is not in the vault")
            return False
        c, r = self.slot_of[name]
        stand = [f"rack{r}_base", f"rack{r}_lip", f"rack{r}_back"]
        holder = ["transfer_nest", "transfer_lip", "transfer_back"]
        self.say(f"   Kawasaki: collecting {name} (stand {r + 1}, slot {c + 1})")
        ok = self.pick(name, slot_rest(c, r), TASK["slot_lift"], stand)
        if ok:
            self.in_stand.discard(name)
            self.say("   Kawasaki: standing it in the hand-off holder")
            g = grasp_from_rest(holder_rest())
            ok = self.place(holder_rest(), self._low_lift(g), holder)
        ok = ok and self.move_to(self.home)
        return ok

    def put_back(self, name):
        """Return mount -> the card's own slot, then back to the ready pose."""
        c, r = self.slot_of[name]
        stand = [f"rack{r}_base", f"rack{r}_lip", f"rack{r}_back"]
        mount = [o.name for o in self.world.obstacles if o.name.startswith("mount_return")]
        self.say(f"   Kawasaki: taking {name} back to stand {r + 1}, slot {c + 1}")
        g = grasp_from_rest(return_rest())
        ok = self.pick(name, return_rest(), self._low_lift(g), mount)
        ok = ok and self.place(slot_rest(c, r), TASK["slot_lift"], stand)
        if ok:
            self.in_stand.add(name)
        ok = ok and self.move_to(self.home)
        return ok