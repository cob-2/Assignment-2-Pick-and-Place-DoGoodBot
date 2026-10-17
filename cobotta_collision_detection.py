"""COBOTTA collision detection (point cloud vs link ellipsoids), motion control and a test cell. Explained in CODE_GUIDE.md."""

import os
from contextlib import contextmanager
from math import pi

import numpy as np
import swift
import spatialgeometry as geometry
from spatialmath import SE3
from roboticstoolbox import RevoluteDH, DHRobot, jtraj

try:
    from ir_support.robots.DHRobot3D import DHRobot3D
except ImportError:
    from ir_support import DHRobot3D


# ============================================================================
# CONFIG
# ============================================================================
BENCH_H = 0.75
NEST_H = 0.040

CONFIG = dict(

    robot=dict(
        xyz=(0.0, 0.0, BENCH_H + 0.060),
        rpy=(0, 0, 0),
        home_deg=(0, 10, 100, 0, 70, 0),
        pedestal=dict(radius=0.070, height=0.060),
        link_radii=(0.055, 0.050, 0.045, 0.036, 0.028, 0.022),
        link_boxes=(
            ((-0.0181, +0.0000, +0.0540), (0.0839, 0.0678, 0.0540)),
            ((+0.0000, +0.0165, +0.0042), (0.0492, 0.0545, 0.0533)),
            ((+0.0843, +0.0120, +0.0903), (0.1172, 0.0483, 0.0318)),
            ((-0.0072, -0.0022, +0.0274), (0.0380, 0.0353, 0.0606)),
            ((-0.0000, -0.0307, -0.0002), (0.0307, 0.0578, 0.0308)),
            ((+0.0000, +0.0016, +0.0055), (0.0293, 0.0309, 0.0330)),
            ((+0.0000, -0.0000, +0.0159), (0.0276, 0.0306, 0.0194)),
        ),
        tool_length=0.060,
        tool_radius=0.016,
    ),

    surfaces=dict(
        bench=dict(size=(1.10, 0.90, 0.04), xyz=(0.0, 0.0, BENCH_H - 0.02),
                   colour=(0.42, 0.44, 0.48, 1.0), ignore_links=(0, 1)),
        plinth=dict(size=(1.00, 0.80, BENCH_H - 0.04), xyz=(0.0, 0.0, (BENCH_H - 0.04) / 2),
                    colour=(0.22, 0.24, 0.28, 1.0), ignore_links=(0, 1)),
    ),

    zones=dict(
        collection=dict(xyz=(0.20, -0.14, BENCH_H), size=(0.13, 0.17, NEST_H),
                        colour=(0.15, 0.55, 0.55, 1.0)),
        inspection=dict(xyz=(0.02, -0.23, BENCH_H), size=(0.15, 0.15, NEST_H),
                        colour=(0.20, 0.45, 0.70, 1.0)),
        packing=dict(xyz=(-0.17, -0.14, BENCH_H), size=(0.17, 0.19, NEST_H),
                     colour=(0.55, 0.40, 0.20, 1.0)),
        delivery=dict(xyz=(-0.08, 0.23, BENCH_H), size=(0.19, 0.19, NEST_H),
                      colour=(0.20, 0.60, 0.30, 1.0)),
        deposit=dict(xyz=(0.13, 0.20, BENCH_H), size=(0.13, 0.13, NEST_H),
                     colour=(0.70, 0.45, 0.15, 1.0)),
        staging=dict(xyz=(0.21, 0.02, BENCH_H), size=(0.11, 0.11, NEST_H),
                     colour=(0.45, 0.45, 0.50, 1.0)),
    ),

    mount=dict(zones=("collection", "deposit"),
               shelf=(0.100, 0.044, 0.008),
               rib=(0.100, 0.005, 0.030),
               slot_clearance=0.0015,
               colour=(0.30, 0.34, 0.40, 1.0)),

    card=dict(size=(0.08025, 0.00625, 0.13575),
              colour=(0.85, 0.82, 0.35, 1.0),
              mesh=None, mesh_scale=(1, 1, 1)),

    box=dict(inner=(0.090, 0.018, 0.150),
             wall=0.003,
             lid=(0.086, 0.016, 0.010),
             handle=(0.020, 0.014, 0.030),
             drop=0.045,
             grip_below_rim=0.020,
             colour=(0.72, 0.58, 0.38, 1.0),
             lid_zone="staging"),

    camera=dict(height=0.32, mast=0.030, body=0.060, offset=0.075,
                colour=(0.10, 0.10, 0.12, 1.0)),

    motion=dict(steps=35,
                check_every=2,
                margin=0.005,
                jaw_margin=0.002,
                replan_tries=15,
                lift=0.110,
                inspect_height=0.110,
                grip_depth=0.015,
                point_spacing=0.012,
                dt=0.03),

    gripper=dict(open=0.015, closed=0.004,
                 finger=(0.014, 0.006, 0.060),
        pad=(0.0311, 0.0139, 0.0239),
        pad_xyz=(0.0000, 0.0069, 0.0481),
                 clearance=0.001),

    intruder=dict(size=(0.07, 0.07, 0.16), xyz=(-0.10, -0.20, BENCH_H + 0.08),
                  colour=(0.85, 0.25, 0.20, 1.0)),
)

MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "models", "cobotta_meshes")


# ============================================================================
# COBOTTA MODEL
# ============================================================================
DH_TABLE = [
    (0.1800,  0.0000, -pi / 2, 0.0,    (-150.0, 150.0)),
    (0.0000, -0.1650,  0.0,    pi / 2, (-60.0,  100.0)),
    (0.0200,  0.0120, -pi / 2, pi / 2, (18.0,   140.0)),
    (0.1775,  0.0000,  pi / 2, 0.0,    (-170.0, 170.0)),
    (-0.0645, 0.0000,  pi / 2, pi,     (-95.0,  135.0)),
    (0.0420,  0.0000,  0.0,    0.0,    (-170.0, 170.0)),
]

QTEST_TRANSFORMS = [SE3(0, 0, 0), SE3(0, 0, 0), SE3(0, 0, 0.1800), SE3(0, 0, 0.3450),
                    SE3(-0.012, 0.020, 0.4330), SE3(-0.012, 0, 0.5225),
                    SE3(-0.012, -0.0445, 0.5645)]


def cobotta_links():
    return [RevoluteDH(d=d, a=a, alpha=al, offset=of,
                       qlim=[np.deg2rad(lo), np.deg2rad(hi)])
            for d, a, al, of, (lo, hi) in DH_TABLE]


class COBOTTA(DHRobot3D):
    def __init__(self, base=SE3()):
        super().__init__(
            cobotta_links(),
            link3D_names=dict(link0="base_link", link1="J1", link2="J2",
                              link3="J3", link4="J4", link5="J5", link6="J6"),
            link3d_dir=MESH_DIR, name="COBOTTA",
            qtest=[0] * 6, qtest_transforms=[T.A for T in QTEST_TRANSFORMS])
        self.base = base
        self.q = np.zeros(6)


class ParallelGripper:
    def __init__(self, cfg):
        c = (0.25, 0.27, 0.30, 1.0)
        self.base = geometry.Mesh(os.path.join(MESH_DIR, "gripper_base.dae"), color=c)
        self.left = geometry.Mesh(os.path.join(MESH_DIR, "left_finger.dae"), color=c)
        self.right = geometry.Mesh(os.path.join(MESH_DIR, "right_finger.dae"), color=c)
        self.opening = cfg["open"]

    def add_to_env(self, env):
        for m in (self.base, self.left, self.right):
            env.add(m)

    def update(self, T, opening=None):
        if opening is not None:
            self.opening = opening
        self.base.T = T.A
        self.left.T = (T * SE3(0, self.opening, 0)).A
        self.right.T = (T * SE3(0, -self.opening, 0)).A


# ============================================================================
# COLLISION CHECKING (point cloud vs link ellipsoids)
# ============================================================================
def surface_points(size, spacing):
    hx, hy, hz = np.asarray(size, dtype=float) / 2

    def grid(u, v):
        a = np.linspace(-u, u, max(2, int(np.ceil(2 * u / spacing)) + 1))
        b = np.linspace(-v, v, max(2, int(np.ceil(2 * v / spacing)) + 1))
        A, B = np.meshgrid(a, b)
        return A.ravel(), B.ravel()

    faces = []
    y, z = grid(hy, hz)
    faces += [np.c_[np.full_like(y, s * hx), y, z] for s in (-1, 1)]
    x, z = grid(hx, hz)
    faces += [np.c_[x, np.full_like(x, s * hy), z] for s in (-1, 1)]
    x, y = grid(hx, hy)
    faces += [np.c_[x, y, np.full_like(x, s * hz)] for s in (-1, 1)]
    return np.unique(np.vstack(faces), axis=0)


class Obstacle:
    def __init__(self, name, size, pose, spacing, ignore_links=()):
        self.name = name
        self.half = np.asarray(size, dtype=float) / 2
        self.pose = pose
        self.ignore_links = set(ignore_links)
        self.active = True
        self.local_points = surface_points(size, spacing)
        self._cache_key = None
        self._cache = None

    def points(self):
        key = self.pose.A.tobytes()
        if key != self._cache_key:
            self._cache = self.local_points @ self.pose.R.T + self.pose.t
            self._cache_key = key
        return self._cache


class World:
    def __init__(self, spacing):
        self.obstacles = []
        self.spacing = spacing

    def add_box(self, name, size, pose, ignore_links=()):
        ob = Obstacle(name, size, pose, self.spacing, ignore_links)
        self.obstacles.append(ob)
        return ob

    def get(self, name):
        return next(o for o in self.obstacles if o.name == name)


def _frame_along(a, b):
    d = b - a
    length = float(np.linalg.norm(d))
    if length < 1e-9:
        return np.eye(3), 0.0
    z = d / length
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z)), length


def arm_ellipsoids(robot, cfg, q):
    frames = robot.fkine_all(q)
    boxes = cfg["robot"]["link_boxes"]
    margin = cfg["motion"]["margin"]
    out = []
    for i, (centre, half) in enumerate(boxes[:len(frames)]):
        T = frames[i]
        out.append((i, (T * SE3(*centre)).t, T.R, np.asarray(half) + margin))
    flange = robot.fkine(q)
    g = cfg["gripper"]
    px, py, pz = g["pad_xyz"]
    pad = np.asarray(g["pad"]) / 2 + cfg["motion"]["jaw_margin"]
    travel = getattr(robot, "gripper_opening", g["open"])
    idx = len(boxes)
    for side in (1, -1):
        c = flange * SE3(px, side * (travel + py), pz)
        out.append((idx, c.t, flange.R, pad))
    return out


def contacts(robot, cfg, world, q):
    found = set()
    for idx, centre, R, radii in arm_ellipsoids(robot, cfg, q):
        for ob in world.obstacles:
            if not ob.active or idx in ob.ignore_links:
                continue
            rel = (ob.points() - centre) @ R
            d = np.sum((rel / radii) ** 2, axis=1)
            if np.any(d < 1.0):
                found.add(f"link{idx}/{ob.name}")
    return found


def first_collision(robot, cfg, world, q, allowed=()):
    new = contacts(robot, cfg, world, q) - set(allowed)
    return sorted(new)[0] if new else None


def path_collision(robot, cfg, world, qs, allowed=()):
    for k, q in enumerate(qs):
        if k % cfg["motion"]["check_every"]:
            continue
        hit = first_collision(robot, cfg, world, q, allowed)
        if hit:
            return k, hit
    return None, None


# ============================================================================
# MOTION
# ============================================================================
class Controller:
    def __init__(self, robot, gripper, env, world, cfg):
        self.robot, self.gripper, self.env = robot, gripper, env
        self.world, self.cfg = world, cfg
        self.carried = []
        self.faults = 0
        self.replans = 0

    @contextmanager
    def ignoring(self, names, links=None):
        obs = [self.world.get(n) for n in names]
        previous = [set(o.ignore_links) for o in obs]
        for o in obs:
            if links is None:
                o.active = False
            else:
                o.ignore_links = o.ignore_links | set(links)
        try:
            yield
        finally:
            held = [ob for _, ob, _ in self.carried]
            for o, p in zip(obs, previous):
                o.ignore_links = p
                o.active = o not in held

    def plan(self, q_goal):
        m = self.cfg["motion"]
        start = contacts(self.robot, self.cfg, self.world, self.robot.q)
        qs = jtraj(self.robot.q, q_goal, m["steps"]).q
        k, hit = path_collision(self.robot, self.cfg, self.world, qs, start)
        if hit is None:
            return qs
        print(f"    collision predicted at step {k} ({hit}) - re-planning")
        rng = np.random.default_rng(1)
        home = np.deg2rad(self.cfg["robot"]["home_deg"])
        def lifted(q):
            out = np.array(q, dtype=float)
            out[1] = float(np.clip(out[1] - 0.6, *self.robot.links[1].qlim))
            out[2] = float(np.clip(out[2] + 0.4, *self.robot.links[2].qlim))
            return out
        candidates = [home, lifted(self.robot.q), lifted(q_goal)]
        for attempt in range(m["replan_tries"]):
            q_mid = (candidates[attempt] if attempt < len(candidates)
                     else np.array([rng.uniform(*l.qlim) for l in self.robot.links]))
            if first_collision(self.robot, self.cfg, self.world, q_mid, start):
                continue
            leg1 = jtraj(self.robot.q, q_mid, m["steps"]).q
            leg2 = jtraj(q_mid, q_goal, m["steps"]).q
            if (path_collision(self.robot, self.cfg, self.world, leg1, start)[1] is None and
                    path_collision(self.robot, self.cfg, self.world, leg2, start)[1] is None):
                self.replans += 1
                print("    re-planned around it")
                return np.vstack((leg1, leg2))
        self.faults += 1
        print("    no safe path - holding")
        return None

    def move_to(self, q_goal):
        qs = self.plan(q_goal)
        if qs is None:
            return False
        for q in qs:
            self.robot.q = q
            self.refresh()
            self.env.step(self.cfg["motion"]["dt"])
        return True

    def move_to_pose(self, T, mask=None):
        home = np.deg2rad(self.cfg["robot"]["home_deg"])
        for q0 in (self.robot.q, home):
            sol = self.robot.ikine_LM(T, q0=q0, joint_limits=True,
                                      mask=mask, slimit=40, ilimit=200)
            if sol.success:
                return self.move_to(sol.q)
        self.faults += 1
        print("    IK failed")
        return False

    def refresh(self):
        self.robot._update_3dmodel()
        T = self.robot.fkine(self.robot.q)
        self.gripper.update(T)
        self.robot.gripper_opening = self.gripper.opening
        for shape, ob, grip in self.carried:
            shape.T = (T * grip).A
            ob.pose = T * grip

    def grasp(self, shape, ob, width=None):
        T = self.robot.fkine(self.robot.q)
        self.carried.append((shape, ob, T.inv() * SE3(shape.T)))
        ob.active = False
        g = self.cfg["gripper"]
        if width is not None:
            self.gripper.opening = float(np.clip(width / 2 + g["clearance"],
                                                 g["closed"], g["open"]))
        self.refresh()
        self.env.step(self.cfg["motion"]["dt"])

    def release(self, place_T=None):
        primary = self.carried[0][0].T.copy() if self.carried else None
        for shape, ob, _ in self.carried:
            if place_T is not None:
                offset = SE3(np.linalg.inv(primary) @ shape.T)
                world_T = place_T * offset
                shape.T = world_T.A
                ob.pose = world_T
            ob.active = True
        self.carried.clear()
        self.gripper.opening = self.cfg["gripper"]["open"]
        self.refresh()
        self.env.step(self.cfg["motion"]["dt"])


# ============================================================================
# SCENE
# ============================================================================
TOOL_DOWN = SE3.Rx(pi)


class Cell:
    def __init__(self, cfg=CONFIG):
        self.cfg = cfg
        self.world = World(cfg["motion"]["point_spacing"])
        self.env = swift.Swift()
        self.shapes = {}

    def _box(self, name, size, pose, colour, ignore_links=()):
        shape = geometry.Cuboid(scale=list(size), pose=pose, color=colour)
        self.env.add(shape)
        self.shapes[name] = shape
        self.world.add_box(name, size, pose, ignore_links)
        return shape

    def build(self, headless=False):
        c = self.cfg
        self.env.launch(realtime=not headless, headless=headless)

        for name, s in c["surfaces"].items():
            self._box(name, s["size"], SE3(*s["xyz"]), s["colour"], s.get("ignore_links", ()))

        ped = c["robot"]["pedestal"]
        self._box("pedestal", (ped["radius"] * 2, ped["radius"] * 2, ped["height"]),
                  SE3(c["robot"]["xyz"][0], c["robot"]["xyz"][1], BENCH_H + ped["height"] / 2),
                  (0.30, 0.32, 0.36, 1.0), ignore_links=(0, 1, 2))

        for name, z in c["zones"].items():
            self._box(f"nest_{name}", z["size"],
                      SE3(z["xyz"][0], z["xyz"][1], z["xyz"][2] + z["size"][2] / 2),
                      z["colour"])

        cam, zi = c["camera"], c["zones"]["inspection"]["xyz"]
        top = zi[2] + c["zones"]["inspection"]["size"][2]
        self._box("camera_mast", (cam["mast"], cam["mast"], cam["height"]),
                  SE3(zi[0], zi[1] - 2 * cam["offset"], top + cam["height"] / 2), cam["colour"])
        self._box("camera_head", (cam["body"], cam["body"], cam["body"]),
                  SE3(zi[0], zi[1] - cam["offset"], top + cam["height"]), cam["colour"])

        m = c["mount"]
        self.mount_h = m["shelf"][2]
        slot = c["card"]["size"][1] / 2 + m["slot_clearance"] + m["rib"][1] / 2
        for zone in m["zones"]:
            shelf = self.resting_pose(zone, m["shelf"][2])
            self._box(f"mount_{zone}", m["shelf"], shelf, m["colour"])
            for side in (1, -1):
                tag = "py" if side > 0 else "my"
                self._box(f"mount_{zone}_rib_{tag}", m["rib"],
                          shelf * SE3(0, side * slot, (m["shelf"][2] + m["rib"][2]) / 2),
                          m["colour"])

        card = c["card"]
        self.card_pose = self.resting_pose("collection", card["size"][2], dz=self.mount_h)
        if card["mesh"]:
            shape = geometry.Mesh(card["mesh"], pose=self.card_pose,
                                  scale=card["mesh_scale"], color=card["colour"])
            self.env.add(shape)
            self.shapes["card"] = shape
            self.world.add_box("card", card["size"], self.card_pose)
        else:
            self._box("card", card["size"], self.card_pose, card["colour"])

        b = c["box"]
        ix, iy, iz = b["inner"]
        w = b["wall"]
        self.box_outer = (ix + 2 * w, iy + 2 * w, iz + w)
        base = self.resting_pose("packing", w)
        self._box("box_base", (self.box_outer[0], self.box_outer[1], w), base, b["colour"])
        for side in (1, -1):
            tag = "py" if side > 0 else "my"
            self._box(f"box_wall_{tag}", (self.box_outer[0], w, iz),
                      base * SE3(0, side * (iy + w) / 2, (w + iz) / 2), b["colour"])
        for side in (1, -1):
            tag = "px" if side > 0 else "mx"
            self._box(f"box_wall_{tag}", (w, iy, iz),
                      base * SE3(side * (ix + w) / 2, 0, (w + iz) / 2), b["colour"])
        self.box_parts = [n for n in self.shapes if n.startswith("box_")]

        self.lid_size, self.handle_size = b["lid"], b["handle"]
        lid_rest = self.resting_pose(b["lid_zone"], self.lid_size[2])
        self._box("box_lid", self.lid_size, lid_rest, b["colour"])
        self._box("box_handle", self.handle_size,
                  lid_rest * SE3(0, 0, (self.lid_size[2] + self.handle_size[2]) / 2),
                  (0.60, 0.48, 0.30, 1.0))

        self.robot = COBOTTA(SE3(*c["robot"]["xyz"]) * SE3.RPY(list(c["robot"]["rpy"])))
        self.robot.q = np.deg2rad(c["robot"]["home_deg"])
        self.robot.add_to_env(self.env)
        self.gripper = ParallelGripper(c["gripper"])
        self.gripper.add_to_env(self.env)

        self.ctrl = Controller(self.robot, self.gripper, self.env, self.world, c)
        self.ctrl.refresh()
        self.env.step(0.0)
        return self

    def nest_top(self, zone):
        z = self.cfg["zones"][zone]
        return z["xyz"][2] + z["size"][2]

    def resting_pose(self, zone, height, dz=0.0):
        z = self.cfg["zones"][zone]["xyz"]
        return SE3(z[0], z[1], self.nest_top(zone) + height / 2 + dz)

    def grasp_pose(self, zone, object_height, dz=0.0, xy=(0.0, 0.0), depth=None):
        r = self.cfg["robot"]
        z = self.cfg["zones"][zone]["xyz"]
        depth = self.cfg["motion"]["grip_depth"] if depth is None else depth
        height = self.nest_top(zone) + object_height + dz + r["tool_length"] - depth
        return SE3(z[0] + xy[0], z[1] + xy[1], height) * TOOL_DOWN

    def approach_pose(self, zone, object_height=0.0, dz=0.0, xy=(0.0, 0.0), depth=None):
        T = self.grasp_pose(zone, object_height, dz, xy, depth)
        return SE3(0, 0, self.cfg["motion"]["lift"]) * T

    def inspect_pose(self, flipped=False):
        z = self.cfg["zones"]["inspection"]["xyz"]
        T = SE3(z[0], z[1], self.nest_top("inspection") + self.cfg["motion"]["inspect_height"]
                + self.cfg["robot"]["tool_length"]) * TOOL_DOWN
        return T * SE3.Rz(pi) if flipped else T

    def occupants(self, zone):
        z = self.cfg["zones"][zone]["xyz"]
        half = np.asarray(self.cfg["zones"][zone]["size"][:2]) / 2
        held = [ob.name for _, ob, _ in self.ctrl.carried]
        out = []
        for ob in self.world.obstacles:
            if ob.name.startswith("nest_") or ob.name in held:
                continue
            d = np.abs(np.array([z[0], z[1]]) - ob.pose.t[:2]) - (ob.half[:2] + half)
            if np.all(d < 0.0) and ob.pose.t[2] > z[2]:
                out.append(ob.name)
        return out

    def contents(self, name):
        host = self.world.get(name)
        held = [ob.name for _, ob, _ in self.ctrl.carried]
        out = []
        for ob in self.world.obstacles:
            if ob is host or ob.name.startswith("nest_") or ob.name in held:
                continue
            gap = np.abs(ob.pose.t[:2] - host.pose.t[:2]) - (host.half[:2] + ob.half[:2])
            if np.all(gap < 0) and ob.pose.t[2] > host.pose.t[2] - host.half[2]:
                out.append(ob.name)
        return out

    def pick(self, name, zone, object_height, dz=0.0, xy=(0.0, 0.0), depth=None,
             grip=None, width=None):
        ctrl = self.ctrl
        grip = grip or name
        width = float(self.world.get(grip).half[1] * 2) if width is None else float(width)
        ok = ctrl.move_to_pose(self.approach_pose(zone, object_height, dz, xy, depth))
        riders = self.contents(name)
        with ctrl.ignoring([name, grip, f"nest_{zone}"] + self.occupants(zone)):
            ok &= ctrl.move_to_pose(self.grasp_pose(zone, object_height, dz, xy, depth))
            ctrl.grasp(self.shapes[name], self.world.get(name), width)
            for r in riders:
                ctrl.grasp(self.shapes[r], self.world.get(r))
            ok &= ctrl.move_to_pose(self.approach_pose(zone, object_height, dz, xy, depth))
        return ok

    def place(self, zone, object_height, dz=0.0, xy=(0.0, 0.0), depth=None, resting=None):
        ctrl = self.ctrl
        names = [ob.name for _, ob, _ in ctrl.carried]
        ok = ctrl.move_to_pose(self.approach_pose(zone, object_height, dz, xy, depth))
        with ctrl.ignoring(names + [f"nest_{zone}"] + self.occupants(zone)):
            ok &= ctrl.move_to_pose(self.grasp_pose(zone, object_height, dz, xy, depth))
            rest = resting if resting is not None else self.resting_pose(zone, object_height, dz)
            ctrl.release(rest * SE3(xy[0], xy[1], 0) if xy != (0.0, 0.0) else rest)
            ok &= ctrl.move_to_pose(self.approach_pose(zone, object_height, dz, xy, depth))
        return ok

    def run_cycle(self, reject=False):
        cfg, ctrl = self.cfg, self.ctrl
        card_h = cfg["card"]["size"][2]
        home = np.deg2rad(cfg["robot"]["home_deg"])

        print("1. collect the card")
        self.pick("card", "collection", card_h, dz=self.mount_h)

        print("2. present the front face to the camera")
        ctrl.move_to_pose(self.inspect_pose())
        self.env.step(0.4)

        print("3. flip and present the back face")
        q = self.robot.q.copy()
        q[5] = float(np.clip(q[5] + pi, *self.robot.links[5].qlim))
        ctrl.move_to(q)
        self.env.step(0.4)

        if reject:
            print("4. card rejected - into the deposit mount")
            self.place("deposit", card_h, dz=self.mount_h)
            ctrl.move_to(home)
            print(f"cycle complete - {ctrl.replans} re-plan(s), {ctrl.faults} fault(s)")
            return

        print("4. pack the card into the holder")
        b = cfg["box"]
        lid_h, handle_h = self.lid_size[2], self.handle_size[2]
        self.place("packing", card_h, dz=b["wall"] + b["drop"],
                   resting=self.resting_pose("packing", card_h, dz=b["wall"]))

        print("5. cap it with the lid")
        self.pick("box_lid", b["lid_zone"], lid_h + handle_h, depth=0.012,
                  grip="box_handle")
        self.place("packing", lid_h + handle_h, dz=b["wall"] + card_h, depth=0.012,
                   resting=self.resting_pose("packing", lid_h, dz=b["wall"] + card_h))

        print("6. deliver the holder")
        below = b["grip_below_rim"]
        self.pick("box_base", "packing", self.box_outer[2], depth=below,
                  grip="box_wall_py", width=self.box_outer[1])
        self.place("delivery", self.box_outer[2], depth=below,
                   resting=self.resting_pose("delivery", b["wall"]))

        ctrl.move_to(home)
        print(f"cycle complete - {ctrl.replans} re-plan(s), {ctrl.faults} fault(s)")


    def drop_intruder(self):
        i = self.cfg["intruder"]
        self._box("intruder", i["size"], SE3(*i["xyz"]), i["colour"])
        self.env.step(0.0)
        print("intruder dropped into the cell")


# ============================================================================
# ENTRY POINT
# ============================================================================
def main():
    import argparse
    ap = argparse.ArgumentParser(description="COBOTTA inspection and packaging cell")
    ap.add_argument("--reject", action="store_true", help="fail inspection, use the deposit zone")
    ap.add_argument("--intruder", action="store_true", help="drop an object in the way")
    ap.add_argument("--headless", action="store_true")
    a = ap.parse_args()

    cell = Cell().build(headless=a.headless)
    if a.intruder:
        cell.drop_intruder()
    cell.run_cycle(reject=a.reject)
    if not a.headless:
        cell.env.hold()


if __name__ == "__main__":
    main()
