"""Kawasaki RS007N (+ OnRobot RG2) collision detection: scene point cloud vs link ellipsoids.

Same idea and names as cobotta_collision_detection.py so the two arms read the same:
every scene object becomes a cloud of surface points, the arm becomes a set of
ellipsoids, and a point p is inside an ellipsoid (centre c, axes R, radii r) when

        sum( ((p - c) @ R / r) ** 2 ) < 1

Difference from the COBOTTA version: the ellipsoids are not guessed from link
radii, they were fitted to the real Kawasaki and RG2 meshes (each one is the
smallest PCA-aligned ellipsoid that contains every vertex of its part). The
long links (base, upper arm, forearm) are split into 2-3 ellipsoids so the
fit stays tight. A carried slab is checked as an exact box.

Run it on its own for a self-test:
    python kawasaki_collision_detection.py            # checks every slot, hand-off and return pose
    python kawasaki_collision_detection.py --demo     # shows what a detected collision looks like
"""

import importlib
from contextlib import contextmanager

import numpy as np
from spatialmath import SE3
from roboticstoolbox import jtraj


# ============================================================================
# CONFIG
# ============================================================================
ARM = "A_vault"

CONFIG = dict(
    margin=0.005,           # extra clearance added to every arm ellipsoid (m)
    jaw_margin=0.002,       # smaller clearance for the RG2 finger parts (they work close to slabs)
    carry_margin=0.003,     # clearance around a carried slab
    point_spacing=0.012,    # spacing of the scene surface points (m)
    steps=40,               # jtraj steps when planning a move
    check_every=2,          # check every n-th step of a path
    reach=1.30,             # ignore scene objects further than this from the base (m)
    skip_groups=("room", "markers"),
    stand_on_tol=0.012,     # objects whose top is this close to the base height are what the arm stands on
)


# ============================================================================
# ELLIPSOIDS FITTED TO THE MESHES
# ============================================================================
# Each entry: (centre, R, radii) in the frame of the part.
#   - LINK_ELLIPSOIDS[i] are in DH frame i (frame 0 = robot base), the same frame
#     ir_support's DHRobot3D uses to place mesh RS007N_J{i}.
#   - RG2_ELLIPSOIDS are in each RG2 part's own frame: "base" in the flange frame,
#     the knuckles and fingers in the frames from RS007N._rg2_link_poses().
# R's columns are the ellipsoid axes, radii are the semi-axis lengths (m).
def E(centre, R, radii):
    R = np.array(R, dtype=float)
    U, _, Vt = np.linalg.svd(R)          # re-orthonormalise after rounding
    return np.array(centre, dtype=float), U @ Vt, np.array(radii, dtype=float)


LINK_ELLIPSOIDS = {
    0: [E((0.0033, 0.0123, 0.0974), ((-0.0046, 0.9972, 0.0752), (-0.8680, -0.0413, 0.4948), (0.4965, -0.0630, 0.8657)), (0.1948, 0.1286, 0.1486)),
        E((-0.0083, -0.1225, 0.0814), ((-0.2488, 0.9257, -0.2850), (-0.4255, -0.3688, -0.8264), (-0.8701, -0.0843, 0.4856)), (0.1659, 0.1401, 0.1547))],
    1: [E((0.0152, -0.0715, 0.0059), ((0.0932, -0.3495, 0.9323), (0.7295, -0.6133, -0.3028), (0.6776, 0.7083, 0.1978)), (0.1729, 0.1626, 0.1436))],
    2: [E((-0.0198, 0.0002, 0.1195), ((0.9968, 0.0094, 0.0799), (0.0106, -0.9999, -0.0136), (0.0797, 0.0144, -0.9967)), (0.1123, 0.0873, 0.0726)),
        E((-0.1811, -0.0043, 0.1173), ((0.9942, -0.0472, -0.0964), (0.0084, 0.9294, -0.3690), (0.1070, 0.3661, 0.9244)), (0.1197, 0.0810, 0.0883)),
        E((-0.3324, 0.0011, 0.1064), ((-0.8278, 0.0067, 0.5610), (0.0173, -0.9991, 0.0375), (0.5608, 0.0408, 0.8270)), (0.1044, 0.0791, 0.1093))],
    3: [E((0.0024, 0.0052, 0.0020), ((0.0172, -0.1376, 0.9903), (-0.3937, 0.9096, 0.1332), (-0.9191, -0.3921, -0.0386)), (0.1363, 0.1220, 0.1030))],
    4: [E((-0.0063, -0.1914, -0.0006), ((0.0392, 0.7298, -0.6825), (-0.0376, -0.6815, -0.7309), (-0.9985, 0.0543, 0.0007)), (0.0867, 0.1236, 0.1132)),
        E((0.0042, -0.0442, -0.0000), ((-0.3502, 0.0139, -0.9366), (-0.9360, 0.0331, 0.3505), (0.0359, 0.9994, 0.0014)), (0.1167, 0.0910, 0.0861))],
    5: [E((-0.0019, 0.0001, 0.0162), ((-0.1193, -0.9925, 0.0284), (0.0031, -0.0290, -0.9996), (0.9929, -0.1191, 0.0065)), (0.0867, 0.0636, 0.0513))],
    6: [E((0.0000, -0.0000, -0.0025), ((0.9661, -0.2581, 0.0002), (0.2581, 0.9661, -0.0002), (-0.0002, 0.0002, 1.0000)), (0.0224, 0.0224, 0.0056))],
}

RG2_ELLIPSOIDS = {
    "base": E((0.0005, -0.0001, 0.0738), ((0.0153, -0.0015, 0.9999), (0.0104, 0.9999, 0.0014), (-0.9998, 0.0104, 0.0153)), (0.1037, 0.0526, 0.0521)),
    "L_outer": E((-0.0003, -0.0172, 0.0174), ((0.0032, 0.9990, -0.0453), (-0.6766, 0.0355, 0.7355), (0.7364, 0.0282, 0.6760)), (0.0553, 0.0164, 0.0223)),
    "R_outer": E((-0.0003, -0.0172, 0.0174), ((0.0032, 0.9990, -0.0453), (-0.6766, 0.0355, 0.7355), (0.7364, 0.0282, 0.6760)), (0.0553, 0.0164, 0.0223)),
    "L_inner": E((-0.0006, -0.0193, 0.0194), ((0.0089, 1.0000, -0.0018), (0.6893, -0.0074, -0.7245), (-0.7244, 0.0052, -0.6893)), (0.0471, 0.0235, 0.0090)),
    "R_inner": E((-0.0006, -0.0193, 0.0194), ((0.0089, 1.0000, -0.0018), (0.6893, -0.0074, -0.7245), (-0.7244, 0.0052, -0.6893)), (0.0471, 0.0235, 0.0090)),
    "L_finger": E((-0.0009, 0.0092, 0.0238), ((-0.0212, -0.9998, -0.0058), (0.3047, -0.0009, -0.9524), (0.9522, -0.0219, 0.3047)), (0.0414, 0.0150, 0.0130)),
    "R_finger": E((-0.0009, 0.0092, 0.0238), ((-0.0212, -0.9998, -0.0058), (0.3047, -0.0009, -0.9524), (0.9522, -0.0219, 0.3047)), (0.0414, 0.0150, 0.0130)),
}

GRIPPER_LINK = 7     # link index used for the RG2 when an obstacle lists ignore_links


# ============================================================================
# SCENE POINT CLOUD
# ============================================================================
def surface_points(size, spacing):
    """Points on the six faces of a box of the given size, centred on the origin."""
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
        self._lo = self._hi = None

    def points(self):
        key = self.pose.A.tobytes()
        if key != self._cache_key:
            self._cache = self.local_points @ self.pose.R.T + self.pose.t
            self._lo, self._hi = self._cache.min(0), self._cache.max(0)
            self._cache_key = key
        return self._cache

    def near(self, centre, radius):
        """Cheap test: could anything within 'radius' of 'centre' touch this obstacle?"""
        self.points()
        gap = np.maximum(0.0, np.maximum(self._lo - centre, centre - self._hi))
        return float(np.linalg.norm(gap)) < radius


class World:
    def __init__(self, spacing=CONFIG["point_spacing"]):
        self.obstacles = []
        self.spacing = spacing

    def add_box(self, name, size, pose, ignore_links=()):
        ob = Obstacle(name, size, pose, self.spacing, ignore_links)
        self.obstacles.append(ob)
        return ob

    def get(self, name):
        return next(o for o in self.obstacles if o.name == name)

    def names(self):
        return [o.name for o in self.obstacles]

    @contextmanager
    def ignoring(self, names):
        """Temporarily switch obstacles off, e.g. the slab being picked and its stand."""
        obs = [self.get(n) for n in names if n in self.names()]
        previous = [o.active for o in obs]
        for o in obs:
            o.active = False
        try:
            yield
        finally:
            for o, p in zip(obs, previous):
                o.active = p


def spec_box(spec):
    """A scene_config object as (size, pose); cylinders and spheres become their bounding box."""
    pose = SE3(*spec["xyz"]) * SE3.RPY(list(spec.get("rpy", (0, 0, 0))))
    if spec["type"] == "cuboid":
        return tuple(spec["size"]), pose
    if spec["type"] == "cylinder":
        r, h = spec["radius"], spec["length"]
        return (2 * r, 2 * r, h), pose
    if spec["type"] == "sphere":
        r = spec["radius"]
        return (2 * r, 2 * r, 2 * r), pose
    return None, None


def world_from_scene(cfg, arm=ARM, spacing=None, include_cards=True, extra=()):
    """Build the obstacle cloud from scene_config.

    Objects the arm stands on (top at or below its base) are ignored by the base
    and turret (links 0 and 1), as in the COBOTTA checker. Objects out of reach are skipped.
    """
    spacing = CONFIG["point_spacing"] if spacing is None else spacing
    world = World(spacing)
    base = np.asarray(cfg.ROBOTS[arm]["base_xyz"], dtype=float)
    specs = cfg.resolved_static() + (cfg.card_objects() if include_cards else []) + list(extra)
    for spec in specs:
        if spec["group"] in CONFIG["skip_groups"]:
            continue
        size, pose = spec_box(spec)
        if size is None:
            continue
        half = np.asarray(size) / 2
        gap = np.maximum(0.0, np.abs(pose.t - base) - np.linalg.norm(half))
        if np.linalg.norm(gap) > CONFIG["reach"]:
            continue
        top = pose.t[2] + half[2]
        under_arm = top <= base[2] + CONFIG["stand_on_tol"]
        world.add_box(spec["name"], size, pose, (0, 1) if under_arm else ())
    return world


# ============================================================================
# ARM ELLIPSOIDS
# ============================================================================
def link_frames(robot, q):
    """World pose of DH frame 0..n (frame 0 = base) as 4x4 arrays."""
    if hasattr(robot, "_get_transforms"):                 # ir_support DHRobot3D
        return [np.asarray(T) for T in robot._get_transforms(q)]
    return [T.A for T in robot.fkine_all(q)]


def gripper_frames(robot, q, opening=None):
    """World pose of each RG2 part, or {} if the robot has no RG2 model."""
    if not hasattr(robot, "flange_pose") or not hasattr(robot, "_rg2_link_poses"):
        return {}
    angle = robot._rg2_angle
    if opening is not None and hasattr(robot, "_lut_gap"):
        angle = float(np.interp(opening, robot._lut_gap, robot._lut_angle))
    F = robot.flange_pose(q)
    out = {"base": F.A}
    for name, P in robot._rg2_link_poses(angle).items():
        out[name] = (F * P).A
    return out


def arm_ellipsoids(robot, q, opening=None, margin=None, jaw_margin=None):
    """Every ellipsoid in world coordinates: list of (link index, label, centre, R, radii)."""
    margin = CONFIG["margin"] if margin is None else margin
    jaw = CONFIG["jaw_margin"] if jaw_margin is None else jaw_margin
    out = []
    for i, T in enumerate(link_frames(robot, q)):
        for k, (c, R, r) in enumerate(LINK_ELLIPSOIDS.get(i, [])):
            out.append((i, f"link{i}" + (f".{k}" if len(LINK_ELLIPSOIDS[i]) > 1 else ""),
                        T[:3, :3] @ c + T[:3, 3], T[:3, :3] @ R, r + margin))
    for name, T in gripper_frames(robot, q, opening).items():
        c, R, r = RG2_ELLIPSOIDS[name]
        pad = margin if name == "base" else jaw
        out.append((GRIPPER_LINK, f"rg2_{name}", T[:3, :3] @ c + T[:3, 3], T[:3, :3] @ R, r + pad))
    return out


def carried_boxes(robot, q, carried):
    """carried: list of (size, T_tcp_object). Returns (label, centre, R, half) in world."""
    if not carried:
        return []
    T = robot.fkine(q)
    out = []
    for k, (size, grip) in enumerate(carried):
        W = (T * grip).A
        out.append((f"carried{k}", W[:3, 3], W[:3, :3],
                    np.asarray(size, dtype=float) / 2 + CONFIG["carry_margin"]))
    return out


# ============================================================================
# COLLISION QUERIES
# ============================================================================
def contacts(robot, world, q, carried=(), opening=None):
    """Set of 'part/obstacle' pairs that touch at joint angles q."""
    found = set()
    for idx, label, centre, R, radii in arm_ellipsoids(robot, q, opening):
        reach = float(radii.max())
        for ob in world.obstacles:
            if not ob.active or idx in ob.ignore_links or not ob.near(centre, reach):
                continue
            rel = (ob.points() - centre) @ R
            if np.any(np.sum((rel / radii) ** 2, axis=1) < 1.0):
                found.add(f"{label}/{ob.name}")
    for label, centre, R, half in carried_boxes(robot, q, carried):
        reach = float(np.linalg.norm(half))
        for ob in world.obstacles:
            if not ob.active or not ob.near(centre, reach):
                continue
            rel = np.abs((ob.points() - centre) @ R)
            if np.any(np.all(rel < half, axis=1)):
                found.add(f"{label}/{ob.name}")
    return found


def first_collision(robot, world, q, allowed=(), carried=(), opening=None):
    new = contacts(robot, world, q, carried, opening) - set(allowed)
    return sorted(new)[0] if new else None


def path_collision(robot, world, qs, allowed=(), carried=(), opening=None, check_every=None):
    """First (step, contact) along a joint path, or (None, None) if it is clear.
    The last step is always checked."""
    every = CONFIG["check_every"] if check_every is None else check_every
    n = len(qs)
    for k, q in enumerate(qs):
        if k % every and k != n - 1:
            continue
        hit = first_collision(robot, world, q, allowed, carried, opening)
        if hit:
            return k, hit
    return None, None


def safe_jtraj(robot, world, q_goal, q_start=None, steps=None, carried=(), opening=None):
    """jtraj from q_start (default: robot.q) to q_goal, checked for collisions.
    Returns (qs, None) if clear, or (None, 'part/obstacle at step k') if blocked.
    Contacts already present at the start are allowed, so a move can leave a touching pose."""
    q0 = np.asarray(robot.q if q_start is None else q_start, dtype=float)
    qs = jtraj(q0, np.asarray(q_goal, dtype=float), steps or CONFIG["steps"]).q
    start = contacts(robot, world, q0, carried, opening)
    k, hit = path_collision(robot, world, qs, start, carried, opening)
    return (qs, None) if hit is None else (None, f"{hit} at step {k}")


def ik_error(robot, q, T):
    """(position error in m, rotation error in rad) of fkine(q) against target T."""
    A = robot.fkine(q)
    dR = A.R.T @ T.R
    ang = float(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)))
    return float(np.linalg.norm(A.t - T.t)), ang


def solve_ik(robot, T, seeds, pos_tol=1e-3, rot_tol=np.radians(1.0)):
    """ikine_LM from each seed in turn; an answer only counts if fkine really lands on T
    (within pos_tol / rot_tol) and every joint is inside its limits. Some toolbox versions
    report success for answers that are a few degrees off, which tilts the gripper."""
    for s in seeds:
        if s is None:
            continue
        q0 = np.asarray(s, dtype=float)
        try:
            sol = robot.ikine_LM(T, q0=q0, joint_limits=True, ilimit=300, slimit=60)
        except TypeError:                                  # older toolbox keyword names
            sol = robot.ikine_LM(T, q0=q0)
        if not getattr(sol, "success", False):
            continue
        q = np.asarray(sol.q, dtype=float)
        lim = np.asarray(robot.qlim)
        if np.any(q < lim[0] - 1e-6) or np.any(q > lim[1] + 1e-6):
            continue
        p_err, r_err = ik_error(robot, q, T)
        if p_err <= pos_tol and r_err <= rot_tol:
            return q
    return None


def straight_path(robot, T_goal, q_start=None, steps=15, seeds=()):
    """Joint path that moves the TCP in a straight line to T_goal (IK at each step,
    seeded with the previous answer). Use it to go into / out of a groove: a jtraj
    lift curves sideways and can scrape the neighbouring slab. Returns None if IK fails."""
    q = np.asarray(robot.q if q_start is None else q_start, dtype=float)
    T0 = robot.fkine(q)
    qs = [q]
    for a in np.linspace(0.0, 1.0, steps)[1:]:
        T = SE3.Rt(T_goal.R, T0.t + a * (T_goal.t - T0.t))
        q_next = solve_ik(robot, T, [q, qs[0], *seeds])
        if q_next is None:
            return None
        if np.max(np.abs(q_next - q)) > np.radians(30):   # a jump = IK switched to another arm configuration
            return None
        q = q_next
        qs.append(q)
    return np.array(qs)


# ============================================================================
# SELF-TEST
# ============================================================================
def load_robot(cfg, arm=ARM):
    spec = cfg.ROBOTS[arm]
    module_name, class_name = spec["loader"].split(":")
    robot = getattr(importlib.import_module(module_name), class_name)()
    robot.base = SE3(*spec["base_xyz"]) * SE3.RPY(list(spec["base_rpy"]))
    if spec.get("q0") is not None:
        robot.q = np.array(spec["q0"], dtype=float)
    return robot


def _ik(robot, T, seeds):
    return solve_ik(robot, T, seeds)


def self_test(demo=False):
    import time
    from math import pi
    import scene_config as cfg

    robot = load_robot(cfg)
    t0 = time.time()
    world = world_from_scene(cfg)
    npts = sum(len(o.points()) for o in world.obstacles)
    print(f"world: {len(world.obstacles)} obstacles, {npts} surface points ({time.time() - t0:.2f} s)")
    import roboticstoolbox as rtb
    print(f"arm: {len(arm_ellipsoids(robot, robot.q))} ellipsoids | roboticstoolbox {rtb.__version__}")

    t0 = time.time()
    hit = contacts(robot, world, robot.q)
    print(f"start pose: {sorted(hit) or 'clear'}  ({(time.time() - t0) * 1000:.0f} ms per check)")

    slab = (cfg.SLAB["w"], cfg.SLAB["t"], cfg.SLAB["h"])
    grip = SE3(0, 0, cfg.SLAB["h"] / 2 - 0.030)      # slab centre relative to the TCP at the grasp
    if hasattr(robot, "set_gripper"):
        robot.set_gripper(cfg.SLAB["t"])
    tool_down = SE3.Rx(pi)
    seeds = [robot.q, [0, 0.5, 1.6, 0, 1.0, 0], [0.4, 0.3, 1.4, 0, 1.2, 0], [-0.4, 0.3, 1.4, 0, 1.2, 0],
             [pi, 0.5, 1.6, 0, 1.0, 0], [pi / 2, 0.5, 1.6, 0, 1.0, 0], [-pi / 2, 0.5, 1.6, 0, 1.0, 0]]

    def stand_of(r):
        return [f"rack{r}_base", f"rack{r}_lip", f"rack{r}_back"]

    # (grasp point, slab to ignore, pieces to ignore while the slab sits in its slot, lift height)
    # Slots lift 20 cm so the slab clears the other stands; the holder and mount only need 8 cm.
    targets = {f"slot c{c} r{r}": (cfg.slot_grasp_xyz(c, r), f"card_c{c}r{r}", stand_of(r), 0.20)
               for r in range(cfg.SLOTS["rows"]) for c in range(cfg.SLOTS["cols"])}
    targets["hand-off holder"] = (cfg.transfer_grasp_xyz(), None,
                                  ["transfer_nest", "transfer_lip", "transfer_back"], 0.08)
    if hasattr(cfg, "mount_grasp_xyz") and "return" in getattr(cfg, "MOUNT", {}).get("at", {}):
        targets["return mount"] = (cfg.mount_grasp_xyz("return"), None,
                                   [n for n in world.names() if n.startswith("mount_return")], 0.08)

    print(f"\n{'target':18s} {'IK':5s} {'straight down to the slab / straight lift with it'}")
    problems = 0
    for name, (xyz, card, stand, lift) in targets.items():
        g = SE3(*xyz) * tool_down
        qa = _ik(robot, SE3(0, 0, lift) * g, seeds)
        qg = _ik(robot, g, ([qa] if qa is not None else []) + seeds)
        if qg is None or qa is None:
            print(f"{name:18s} FAIL  out of reach"); problems += 1
            continue
        with world.ignoring(([card] if card else []) + stand):
            down = straight_path(robot, g, q_start=qa, seeds=seeds)
            up = straight_path(robot, SE3(0, 0, lift) * g, q_start=qg, seeds=seeds)
            if down is None or up is None:
                print(f"{name:18s} FAIL  no straight-line IK"); problems += 1
                continue
            _, hit_down = path_collision(robot, world, down, check_every=1)
            _, hit_up = path_collision(robot, world, up, carried=[(slab, grip)], check_every=1)
            _, hit_j = path_collision(robot, world, jtraj(qg, qa, 15).q, carried=[(slab, grip)], check_every=1)
        verdict = "clear" if not (hit_down or hit_up) else f"HIT {hit_down or hit_up}"
        problems += verdict != "clear"
        note = f"   (a jtraj lift instead would hit: {hit_j})" if hit_j else ""
        print(f"{name:18s} OK    {verdict}{note}")
    print(f"\n{problems} problem(s)")

    if demo:
        print("\n--- demo: poses that must be flagged ---")
        qg = _ik(robot, SE3(*cfg.slot_grasp_xyz(1, 1)) * tool_down, seeds)
        print("at the grasp, slab not ignored        ->", first_collision(robot, world, qg))
        x1, x2 = cfg.slot_xyz(1, 1)[0], cfg.slot_xyz(2, 1)[0]
        between = SE3((x1 + x2) / 2, cfg.RACK["rack_y"][1], cfg.slot_grasp_xyz(1, 1)[2]) * tool_down
        print("gripper dropped between two slabs     ->", first_collision(robot, world, _ik(robot, between, seeds)))
        with world.ignoring(["card_c1r1"]):             # the slab in the gripper is expected
            for joint, label in ((1, "shoulder"), (2, "elbow")):
                for deg in range(1, 61):
                    q = qg.copy(); q[joint] += np.radians(deg)
                    hit = first_collision(robot, world, q)
                    if hit:
                        print(f"{label + f' tilted {deg} deg past grasp':37s} -> {hit}")
                        break
        world.add_box("intruder", (0.10, 0.10, 0.25), SE3(*cfg.slot_xyz(1, 1)) * SE3(0, 0, 0.25))
        q_up = _ik(robot, SE3(0, 0, 0.20) * SE3(*cfg.slot_grasp_xyz(1, 1)) * tool_down, seeds)
        print("box left above the stand, lifting     ->", first_collision(robot, world, q_up))
        qs, why = safe_jtraj(robot, world, q_up, q_start=qg)
        print("safe_jtraj from grasp up into the box ->", "blocked:" if qs is None else "clear", why or "")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Kawasaki RS007N ellipsoid collision checker")
    ap.add_argument("--demo", action="store_true", help="also show detected collisions")
    self_test(demo=ap.parse_args().demo)