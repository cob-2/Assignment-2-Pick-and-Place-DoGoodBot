"""Card Vault workcell - Swift scene builder. Run: python card_vault_scene.py"""

import argparse
import importlib
import os

import numpy as np
import swift
import spatialgeometry as geometry
from spatialmath import SE3

import scene_config as cfg


# ----------------------------------------------------------------------------
# PRIMITIVES
# ----------------------------------------------------------------------------
def pose_of(spec):
    x, y, z = spec["xyz"]
    r, p, yw = spec.get("rpy", (0, 0, 0))
    return SE3(x, y, z) * SE3.RPY([r, p, yw])


def mesh_path(filename):
    if os.path.isabs(filename):
        return filename
    here = os.path.dirname(os.path.abspath(cfg.__file__))
    direct = os.path.join(here, filename)
    return direct if os.path.exists(direct) else os.path.join(here, cfg.MESH_DIR, filename)


def make_shape(spec):
    if spec["type"] == "cuboid":
        return geometry.Cuboid(scale=list(spec["size"]), pose=pose_of(spec),
                               color=spec["color"])
    if spec["type"] == "cylinder":
        return geometry.Cylinder(radius=spec["radius"], length=spec["length"],
                                 pose=pose_of(spec), color=spec["color"])
    if spec["type"] == "sphere":
        return geometry.Sphere(radius=spec["radius"], pose=pose_of(spec),
                               color=spec["color"])
    if spec["type"] == "mesh":
        return geometry.Mesh(filename=mesh_path(spec["filename"]), pose=pose_of(spec),
                             scale=spec.get("scale", (1, 1, 1)), color=spec.get("color"))
    raise ValueError(f"unknown shape type: {spec['type']}")


# ----------------------------------------------------------------------------
# STATIC SCENE
# ----------------------------------------------------------------------------
def build_static(env, skip_groups=()):
    objects = {}
    specs = cfg.resolved_static() + cfg.card_objects()
    for spec in specs:
        if spec["group"] in skip_groups:
            continue
        if spec["group"] == "markers" and not cfg.SHOW_MARKERS:
            continue
        shape = make_shape(spec)
        env.add(shape)
        objects[spec["name"]] = shape
    return objects


# ----------------------------------------------------------------------------
# ROBOTS
# ----------------------------------------------------------------------------
def load_robot(loader):
    if not loader:
        return None
    module_name, class_name = loader.split(":")
    try:
        module = importlib.import_module(module_name)
        return getattr(module, class_name)()
    except Exception as exc:
        print(f"  [placeholder] {loader} not available ({type(exc).__name__}: {exc})")
        return None


def build_robots(env, skip=()):
    robots = {}
    for key, spec in cfg.ROBOTS.items():
        if key in skip:
            continue
        base = SE3(*spec["base_xyz"]) * SE3.RPY(list(spec["base_rpy"]))

        if cfg.SHOW_PEDESTALS:
            px, py, pz = spec["base_xyz"]
            env.add(geometry.Cylinder(radius=cfg.PEDESTAL["radius"],
                                      length=cfg.PEDESTAL["length"],
                                      pose=SE3(px, py, pz + cfg.PEDESTAL["length"] / 2),
                                      color=cfg.C["pedestal"]))

        robot = load_robot(spec["loader"])
        if robot is None:
            h = spec["placeholder_h"]
            env.add(geometry.Cuboid(scale=[0.10, 0.10, h],
                                    pose=base * SE3(0, 0, h / 2),
                                    color=(0.45, 0.45, 0.50, 0.65)))
            robots[key] = None
            continue

        robot.base = base
        if spec["q0"] is not None:
            robot.q = np.array(spec["q0"])
        try:
            robot.add_to_env(env)          # ir_support DHRobot3D
        except AttributeError:
            env.add(robot)                 # plain rtb robot
        robots[key] = robot
        print(f"  [loaded] {spec['label']}")
    return robots


# ----------------------------------------------------------------------------
# SCENE ASSEMBLY
# ----------------------------------------------------------------------------
def build_scene(env, skip_groups=(), skip_robots=()):
    objects = build_static(env, skip_groups=skip_groups)
    robots = build_robots(env, skip=skip_robots)
    return objects, robots


def attach_to_gripper(obj, robot, grip=SE3()):
    """Call every step while an object is carried: T_obj = T_ee * T_grip."""
    obj.T = robot.fkine(robot.q) * grip


# ----------------------------------------------------------------------------
# ENTRY POINT
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Card Vault workcell scene")
    ap.add_argument("--no-robots", action="store_true")
    ap.add_argument("--no-glass", action="store_true")
    ap.add_argument("--hide", nargs="*", default=[], metavar="GROUP")
    ap.add_argument("--browser", default=None)
    args = ap.parse_args()

    skip_groups = set(args.hide)
    if args.no_glass:
        skip_groups.add("enclosure")

    env = swift.Swift()
    env.launch(realtime=True, browser=args.browser)

    objects, robots = build_scene(
        env,
        skip_groups=skip_groups,
        skip_robots=set(cfg.ROBOTS) if args.no_robots else (),
    )
    print(f"scene: {len(objects)} static objects, "
          f"{sum(r is not None for r in robots.values())}/{len(robots)} robot models")

    env.hold()


if __name__ == "__main__":
    main()
