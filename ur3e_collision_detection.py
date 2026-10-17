import numpy as np
from spatialmath import SE3
from roboticstoolbox import jtraj

import kawasaki_collision_detection as kcd
from kawasaki_collision_detection import E, RG2_ELLIPSOIDS, World, spec_box  # noqa: F401  (re-exported)


# ============================================================================
# CONFIG
# ============================================================================
ARM = "C_handover"

CONFIG = dict(
    margin=0.005,           # extra clearance added to every arm ellipsoid (m)
    jaw_margin=0.002,       # smaller clearance for the RG2 finger parts (they work close to the box)
    carry_margin=0.003,     # clearance around a carried box
    steps=40,               # jtraj steps when planning a move
    check_every=2,          # check every n-th step of a path
)


# ============================================================================
# ELLIPSOIDS FITTED TO THE MESHES
# ============================================================================
# Each entry: (centre, R, radii) in DH frame i (frame 0 = robot base), the frame
# ir_support's DHRobot3D places mesh UR3eLink{i} in. R's columns are the axes.
LINK_ELLIPSOIDS = {
    0: [E((-0.0007, 0.0233, 0.0359), ((-0.1220, 0.9916, -0.0431), (0.9709, 0.1103, -0.2126), (-0.2061, -0.0678, -0.9762)), (0.1146, 0.0904, 0.0757))],
    1: [E((0.0011, -0.0067, 0.0038), ((0.0152, 0.1574, -0.9874), (-0.2848, 0.9473, 0.1467), (0.9585, 0.2790, 0.0593)), (0.0739, 0.0896, 0.0656))],
    2: [E((0.2326, 0.0010, 0.1133), ((-0.9951, 0.0323, 0.0938), (0.0207, 0.9923, -0.1219), (-0.0970, -0.1193, -0.9881)), (0.0791, 0.0680, 0.0887)),
        E((0.1269, -0.0004, 0.1250), ((-0.9888, 0.0042, 0.1494), (-0.0079, -0.9997, -0.0245), (0.1492, -0.0254, 0.9885)), (0.0951, 0.0612, 0.0645)),
        E((0.0078, -0.0002, 0.1193), ((-0.7881, 0.6129, 0.0571), (-0.0008, -0.0938, 0.9956), (0.6156, 0.7845, 0.0744)), (0.0805, 0.0764, 0.0513))],
    3: [E((0.1995, -0.0003, 0.0471), ((-0.8354, 0.0051, 0.5497), (-0.0037, 0.9999, -0.0149), (-0.5497, -0.0144, -0.8352)), (0.0720, 0.0458, 0.0471)),
        E((0.1162, -0.0002, 0.0393), ((-0.9392, -0.0049, -0.3432), (-0.0019, 1.0000, -0.0090), (0.3432, -0.0078, -0.9392)), (0.1152, 0.0484, 0.0510)),
        E((0.0047, -0.0002, 0.0305), ((-0.8495, -0.0364, 0.5263), (-0.0317, 0.9993, 0.0181), (-0.5266, -0.0013, -0.8501)), (0.0650, 0.0414, 0.0660))],
    4: [E((-0.0001, -0.0050, 0.0018), ((-0.0127, 0.9990, 0.0431), (0.9339, 0.0273, -0.3565), (-0.3573, 0.0357, -0.9333)), (0.0603, 0.0427, 0.0658))],
    5: [E((0.0001, 0.0050, 0.0018), ((0.0127, -0.9990, -0.0431), (-0.9339, -0.0273, 0.3565), (-0.3573, 0.0357, -0.9333)), (0.0603, 0.0427, 0.0658))],
    6: [E((0.0000, -0.0024, -0.0201), ((-0.0000, -1.0000, 0.0016), (0.9965, -0.0002, -0.0835), (0.0835, 0.0016, 0.9965)), (0.0488, 0.0435, 0.0311))],
}

GRIPPER_LINK = 7     # link index used for the RG2 when an obstacle lists ignore_links


# ============================================================================
# SCENE
# ============================================================================
def world_from_scene(cfg, spacing=None, include_cards=True, extra=()):
    """The obstacle cloud from scene_config, built exactly as for the Kawasaki."""
    return kcd.world_from_scene(cfg, arm=ARM, spacing=spacing,
                                include_cards=include_cards, extra=extra)


# ============================================================================
# ARM ELLIPSOIDS
# ============================================================================
def link_frames(robot, q):
    """World pose of DH frame 0..n (frame 0 = base) as 4x4 arrays."""
    if hasattr(robot, "_get_transforms"):                 # ir_support DHRobot3D
        return [np.asarray(T) for T in robot._get_transforms(q)]
    return [T.A for T in robot.fkine_all(q)]


def gripper_frames(robot, q, opening=None):
    """World pose of each RG2 part, or {} if the robot carries no RG2Gripper.
    opening is the full jaw gap in metres (default: the gripper's current one)."""
    g = getattr(robot, "gripper", None)
    if g is None or not hasattr(g, "_link_poses") or not hasattr(robot, "flange_pose"):
        return {}
    angle = g._angle
    if opening is not None:
        angle = float(np.interp(opening, g._lut_gap, g._lut_angle))
    T = robot.flange_pose(q) * g.mount
    out = {"base": T.A}
    for name, P in g._link_poses(angle).items():
        out[name] = (T * P).A
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
    """carried: list of (size, T_fkine_object). Returns (label, centre, R, half) in world."""
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


# ============================================================================
# SELF-TEST
# ============================================================================
def self_test(demo=False):
    import scene_config as cfg
    import ur3e_outfeed as outfeed

    robot = outfeed.load_robot()
    arm = outfeed.OutfeedArm(None, robot, {}, outfeed.NoGate())
    print(f"arm: {len(arm_ellipsoids(robot, robot.q))} ellipsoids")
    arm.reach_report()
    if demo:
        print("\n--- demo: poses that must be flagged ---")
        # put the closed box on the outfeed pad, where the COBOTTA leaves it
        p = cfg.PACK
        shift = SE3(p["outfeed_xy"][0] - p["holder_xy"][0],
                    p["outfeed_xy"][1] - p["holder_xy"][1], p["pad"][2])
        lid = SE3(0, 0, cfg.holder_lid_xyz()[2] - (cfg.BENCH_H + p["lid"][2] / 2))
        lid_move = SE3(p["holder_xy"][0] - p["lid_xy"][0], p["holder_xy"][1] - p["lid_xy"][1], 0) * lid
        box = [n for n in arm.world.names() if n.startswith("holder_")] + ["box_lid", "box_handle"]
        for n in box:
            ob = arm.world.get(n)
            ob.pose = shift * (lid_move * ob.pose if n.startswith("box_") else ob.pose)
        sp = arm.spots()
        q_grasp = arm.solve(arm.grip_pose(sp["pad"]), q0=arm.home)
        print("at the pad grasp, box not ignored     ->", first_collision(robot, arm.world, q_grasp))
        with arm.world.ignoring(box):
            q = arm.solve(SE3(0, 0, -0.12) * arm.grip_pose(sp["pad"]), q0=q_grasp)
            print("jaws pushed 12 cm below the grasp   ->", first_collision(robot, arm.world, q))
        size = (p["outer"][0], p["outer"][1], p["outer"][2] + p["handle"][2])
        held = SE3(*p["outfeed_xy"], cfg.BENCH_H + p["pad"][2] + size[2] / 2)
        grip = robot.fkine(q_grasp).inv() * held
        q = arm.solve(SE3(0, 0, -0.03) * arm.grip_pose(sp["spur"]), q0=arm.home)
        print("box set down 3 cm too low on the belt ->",
              first_collision(robot, arm.world, q, carried=[(size, grip)]))

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="UR3e + RG2 ellipsoid collision checker")
    ap.add_argument("--demo", action="store_true", help="also show detected collisions")
    self_test(demo=ap.parse_args().demo)
