
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
             drop=0.045,
             grip_below_rim=0.020,
             colour=(0.72, 0.58, 0.38, 1.0)),

    camera=dict(height=0.32, mast=0.030, body=0.060, offset=0.075,
                colour=(0.10, 0.10, 0.12, 1.0)),

    rmrc=dict(
        speed=0.09,            # TCP speed along the straight line (m/s)
        w_max=1.2,             # TCP turn rate cap (rad/s)
        min_time=0.30,         # shortest leg, so tiny moves still ramp (s)
        qd_max=1.50,           # joint speed cap (rad/s)
        manip_min=0.012,       # below this manipulability, damping switches on
        damping_max=0.06,      # largest damping factor (lambda) at a singularity
        tol=0.002,             # final TCP position error accepted (m)
        settle=15,             # extra steps allowed to close the last error
    ),

    self_check=dict(shrink=0.85,   # link capsules are this fraction of the ellipsoid
                    apart=4,       # links this far apart in the chain are tested
                    samples=150),  # random poses used to learn the structural pairs

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
                 pad=(0.0262, 0.0117, 0.0201),   # semi-axes containing the finger mesh
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
def E(centre, R, radii):
    R = np.array(R, dtype=float)
    U, _, Vt = np.linalg.svd(R)          # re-orthonormalise after rounding
    return np.array(centre, dtype=float), U @ Vt, np.array(radii, dtype=float)


# One or more oriented ellipsoids per link, each the smallest PCA-aligned ellipsoid
# that contains every vertex of that link's mesh, measured in the link's own DH
# frame. Long or bent links are split into 2-3 pieces so the fit stays tight. Same
# idea and format as kawasaki_collision_detection.LINK_ELLIPSOIDS.
LINK_ELLIPSOIDS = {
    0: [E((-0.0243, +0.0502, +0.0507), ((-0.9923, +0.1131, +0.0499), (+0.0261, -0.2025, +0.9789), (+0.1208, +0.9727, +0.1980)), (0.1228, 0.0957, 0.0195)),
       E((-0.0098, -0.0030, +0.0694), ((-0.9692, +0.0352, -0.2438), (+0.0288, +0.9991, +0.0295), (+0.2446, +0.0216, -0.9694)), (0.1448, 0.0689, 0.0831)),
       E((-0.0279, -0.0532, +0.0501), ((-0.9922, +0.1207, -0.0322), (-0.0068, +0.2050, +0.9787), (+0.1247, +0.9713, -0.2026)), (0.1135, 0.0907, 0.0148))],   # base_link, 3 piece(s)
    1: [E((-0.0000, +0.0231, +0.0076), ((-0.0000, +1.0000, +0.0000), (-0.8631, -0.0000, -0.5051), (-0.5051, -0.0000, +0.8631)), (0.0774, 0.0610, 0.0908))],   # J1, 1 piece(s)
    2: [E((+0.0098, +0.0089, +0.0832), ((-0.3022, +0.6826, +0.6653), (-0.8850, -0.4602, +0.0701), (+0.3540, -0.5677, +0.7433)), (0.0552, 0.0463, 0.0571)),
       E((+0.0837, +0.0217, +0.0952), ((+0.9908, -0.1267, -0.0480), (-0.1100, -0.9590, +0.2611), (-0.0791, -0.2534, -0.9641)), (0.0658, 0.0534, 0.0394)),
       E((+0.1565, +0.0061, +0.0834), ((+0.3101, -0.7460, +0.5894), (-0.7935, -0.5446, -0.2718), (+0.5237, -0.3834, -0.7608)), (0.0512, 0.0459, 0.0562))],   # J2, 3 piece(s)
    3: [E((-0.0054, -0.0054, +0.0299), ((+0.1875, -0.9620, -0.1983), (-0.0091, +0.2002, -0.9797), (+0.9822, +0.1855, +0.0288)), (0.0867, 0.0501, 0.0520))],   # J3, 1 piece(s)
    4: [E((-0.0001, -0.0313, -0.0030), ((-0.0016, -1.0000, -0.0073), (+0.9951, -0.0023, +0.0984), (-0.0984, -0.0071, +0.9951)), (0.0835, 0.0425, 0.0460))],   # J4, 1 piece(s)
    5: [E((+0.0011, -0.0004, -0.0086), ((-0.0641, -0.9943, -0.0855), (+0.9175, -0.0924, +0.3869), (-0.3926, -0.0536, +0.9182)), (0.0545, 0.0424, 0.0281)),
       E((-0.0005, +0.0060, +0.0262), ((-0.9976, -0.0581, +0.0388), (-0.0686, +0.9218, -0.3815), (-0.0136, -0.3833, -0.9235)), (0.0473, 0.0580, 0.0318))],   # J5, 2 piece(s)
    6: [E((-0.0004, +0.0002, +0.0159), ((-0.0194, -0.9988, +0.0455), (-0.9993, +0.0209, +0.0324), (-0.0333, -0.0448, -0.9984)), (0.0422, 0.0381, 0.0286))],   # J6, 1 piece(s)
    }


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
        self._frames_key = None
        self._frames = None

    def frames(self):
        """Centre, rotation and half-extents of every active obstacle, as three
        stacked arrays. Reading those off a hundred separate SE3 objects costs
        more than the collision arithmetic does, so they are built once and kept
        until something actually moves or is switched off."""
        act = [ob for ob in self.obstacles if ob.active]
        key = b"".join(ob.pose.A.tobytes() for ob in act) + bytes(len(act))
        if key != self._frames_key:
            self._frames = (act,
                            np.array([ob.pose.t for ob in act], dtype=float),
                            np.array([ob.pose.R for ob in act], dtype=float),
                            np.array([ob.half for ob in act], dtype=float))
            self._frames_key = key
        return self._frames

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
    margin = cfg["motion"]["margin"]
    out = []
    for i, T in enumerate(frames):
        for centre, R, radii in LINK_ELLIPSOIDS.get(i, []):
            out.append((i, T.R @ centre + T.t, T.R @ R, radii + margin))
    flange = robot.fkine(q)
    g = cfg["gripper"]
    px, py, pz = g["pad_xyz"]
    pad = np.asarray(g["pad"]) + cfg["motion"]["jaw_margin"]
    travel = getattr(robot, "gripper_opening", g["open"])
    idx = len(frames)
    for side in (1, -1):
        c = flange * SE3(px, side * (travel + py), pz)
        out.append((idx, c.t, flange.R, pad))
    return out


def _unit_sphere(n=48):
    """n roughly even points on the unit sphere (Fibonacci spiral)."""
    k = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * k / n)
    theta = np.pi * (1 + 5 ** 0.5) * k
    return np.column_stack((np.cos(theta) * np.sin(phi),
                            np.sin(theta) * np.sin(phi), np.cos(phi)))


_SPHERE = _unit_sphere()


def ellipsoid_overlap(a, b, shrink=1.0):
    """Do two oriented ellipsoids (centre, R, radii) overlap? Points sampled on each
    surface are tested against the other's inequality, which catches every case
    except two that interpenetrate without either surface sample landing inside -
    impossible here, where the radii are all of a similar order."""
    for (c1, R1, r1), (c2, R2, r2) in ((a, b), (b, a)):
        pts = c1 + (_SPHERE * (np.asarray(r1) * shrink)) @ np.asarray(R1).T
        if np.any(np.sum((((pts - c2) @ np.asarray(R2)) / (np.asarray(r2) * shrink)) ** 2,
                         axis=1) < 1.0):
            return True
    return False


def self_contacts(items, shrink=1.0, apart=2):
    parts = []
    for it in items:
        label = it[1] if isinstance(it[1], str) else f"link{it[0]}"
        parts.append((it[0], label, it[-3], it[-2], np.asarray(it[-1], dtype=float)))
    found = set()
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            if abs(parts[i][0] - parts[j][0]) < apart:
                continue
            reach = (parts[i][4].max() + parts[j][4].max()) * shrink 
            if np.linalg.norm(parts[i][2] - parts[j][2]) > reach:                        continue                                                 
            if ellipsoid_overlap(parts[i][2:], parts[j][2:], shrink):
                found.add(f"{parts[i][1]}~{parts[j][1]}")
    return found


def structural_self_pairs(robot, ellipsoids, shrink=0.85, apart=2, samples=150):
    """Which capsule pairs overlap in *every* pose. Those are an artefact of the
    ellipsoid fit - a wrist ellipsoid that always pokes into the gripper base, say -
    not a fault, so they are subtracted from every self-collision result. Learned
    once per robot from random configurations and cached on it, rather than
    hard-coded, so it stays right if anyone re-fits the ellipsoids."""
    cached = getattr(robot, "_structural_self_pairs", None)
    if cached is not None:
        return cached
    rng = np.random.default_rng(0)
    tally = {}
    for _ in range(samples):
        q = np.array([rng.uniform(*l.qlim) for l in robot.links])
        for pair in self_contacts(ellipsoids(q), shrink, apart):
            tally[pair] = tally.get(pair, 0) + 1
    cached = {p for p, n in tally.items() if n >= 0.95 * samples}
    object.__setattr__(robot, "_structural_self_pairs", cached)
    return cached


def near_obstacles(world, centre, reach):
    """The active obstacles whose box comes within `reach` of a point.

    An obstacle is a cloud of a few thousand points, and the cell holds about a
    hundred of them, so testing every point against every link ellipsoid is
    millions of operations for a result that is almost always "nothing is close".
    The exact distance from a point to an oriented box is three lines of
    arithmetic and is never larger than the distance to that box's cloud, so
    anything this rejects could not have been hit. Same answer, a fifteenth of
    the work."""
    act, centres, rots, halfs = world.frames()
    if not act:
        return []
    local = np.abs(np.einsum("nj,njk->nk", centre - centres, rots))
    gap = np.clip(local - halfs, 0.0, None)
    near = np.linalg.norm(gap, axis=1) <= reach
    return [act[k] for k in np.nonzero(near)[0]]


def contacts(robot, cfg, world, q, skip=()):
    items = arm_ellipsoids(robot, cfg, q)
    sc = cfg.get("self_check", CONFIG["self_check"])
    ignore = structural_self_pairs(robot, lambda qq: arm_ellipsoids(robot, cfg, qq),
                                   sc["shrink"], sc["apart"], sc["samples"])
    found = self_contacts(items, sc["shrink"], sc["apart"]) - ignore
    for idx, centre, R, radii in items:
        reach = float(np.max(radii))
        for ob in near_obstacles(world, centre, reach):
            name = f"link{idx}/{ob.name}"
            if idx in ob.ignore_links or name in skip:
                continue
            pts = ob.points()
            close = pts[np.all(np.abs(pts - centre) <= reach, axis=1)]
            if close.size == 0:
                continue
            rel = (close - centre) @ R
            d = np.sum((rel / radii) ** 2, axis=1)
            if np.any(d < 1.0):
                found.add(name)
    return found


def first_collision(robot, cfg, world, q, allowed=()):
    # A pair already on the allowed list is subtracted from the answer either way,
    # so it is not tested at all - which is what skips the heavy clouds the arm
    # permanently sits against, such as its own base plate in the holder walls.
    allowed = set(allowed)
    new = contacts(robot, cfg, world, q, skip=allowed) - allowed
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

    def rmrc(self, T_goal, speed=None):
        """Resolved-rate motion control of the flange along a straight line to T_goal.

        The TCP follows the line with a quintic profile, so it starts and stops
        smoothly. Each step takes the gap between where the flange is and where it
        should be next step as a spatial velocity xd = [v; w], solves
        qd = J^T (J J^T + lambda^2 I)^-1 xd with lambda > 0 only near a singularity
        (manipulability below manip_min), caps qd and integrates q += qd dt.
        Returns (qs, report), or (None, why) if it could not hold the line."""
        r = self.cfg["rmrc"]
        dt = self.cfg["motion"]["dt"]
        speed = r["speed"] if speed is None else speed
        q = np.asarray(self.robot.q, dtype=float).copy()
        T0 = self.robot.fkine(q)
        dist = float(np.linalg.norm(T_goal.t - T0.t))
        ang = float(abs((T0.inv() * T_goal).angvec()[0]))
        n = max(1, int(np.ceil(max(dist / speed, ang / r["w_max"], r["min_time"]) / dt)))
        lim = np.asarray(self.robot.qlim)
        qs, worst, min_m, err = [], 0.0, np.inf, np.inf
        for k in range(1, n + r["settle"]):
            tau = min(k / n, 1.0)
            sc = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
            p_des = T0.t + sc * (T_goal.t - T0.t)
            R_des = T0.interp(T_goal, sc).R
            T_now = self.robot.fkine(q)
            v = (p_des - T_now.t) / dt
            Re = R_des @ T_now.R.T
            w = 0.5 * np.array([Re[2, 1] - Re[1, 2],
                                Re[0, 2] - Re[2, 0],
                                Re[1, 0] - Re[0, 1]]) / dt
            J = np.asarray(self.robot.jacob0(q))
            m = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
            min_m = min(min_m, m)
            lam = (r["damping_max"] * (1 - (m / r["manip_min"]) ** 2)
                   if m < r["manip_min"] else 0.0)
            qd = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), np.r_[v, w])
            peak = float(np.max(np.abs(qd)))
            if peak > r["qd_max"]:
                qd *= r["qd_max"] / peak
            worst = max(worst, min(peak, r["qd_max"]))
            q = q + qd * dt
            if np.any(q < lim[0]) or np.any(q > lim[1]):
                return None, "joint limit"
            qs.append(q.copy())
            err = float(np.linalg.norm(self.robot.fkine(q).t - T_goal.t))
            if k >= n and err < r["tol"] / 4:
                break
        if err > r["tol"]:
            return None, f"ended {err * 1000:.1f} mm off"
        return np.array(qs), (f"{len(qs)} steps, peak joint speed {worst:.2f} rad/s, "
                              f"min manipulability {min_m:.3f}, error {err * 1000:.2f} mm")

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
        self.place("packing", card_h, dz=b["wall"] + b["drop"],
                   resting=self.resting_pose("packing", card_h, dz=b["wall"]))

        print("5. deliver the holder")
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
