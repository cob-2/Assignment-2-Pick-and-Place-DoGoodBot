"""Geometry audit - prints anything floating, buried or overlapping."""

import itertools
from math import cos, sin

import numpy as np

import scene_config as cfg

SUPPORTS = {0.0: "floor", 0.02: "floor pad", cfg.BENCH_H: "bench", cfg.CONV_TOP + 0.012: "belt"}


def aabb(spec):
    c = np.array(spec["xyz"], dtype=float)
    if spec["type"] == "cuboid":
        h = np.array(spec["size"], dtype=float) / 2
    else:
        r, L = spec["radius"], spec["length"]
        roll = spec.get("rpy", (0, 0, 0))[0]
        h = np.array([r, r, L / 2]) if abs(sin(roll)) < 0.5 else np.array([r, L / 2, r])
    rpy = spec.get("rpy", (0, 0, 0))
    if abs(rpy[0]) > 1e-6 and spec["type"] == "cuboid":        # roll about x
        sy, sz = h[1], h[2]
        h[1] = abs(sy * cos(rpy[0])) + abs(sz * sin(rpy[0]))
        h[2] = abs(sy * sin(rpy[0])) + abs(sz * cos(rpy[0]))
    return c - h, c + h


def main():
    specs = cfg.resolved_static() + cfg.card_objects()
    print("=== vertical support ===")
    for s in specs:
        lo, hi = aabb(s)
        z = lo[2]
        near = min(SUPPORTS, key=lambda k: abs(k - z))
        if abs(near - z) > 0.012:
            print(f"  {s['name']:22s} bottom z={z:+.3f}  (nearest surface {near:.3f} -> gap {z-near:+.3f})")

    print("=== overlaps ===")
    for a, b in itertools.combinations(specs, 2):
        if a["group"] in ("room", "markers") or b["group"] in ("room", "markers"):
            continue
        la, ha = aabb(a)
        lb, hb = aabb(b)
        ov = np.minimum(ha, hb) - np.maximum(la, lb)
        if np.all(ov > 0.004):
            print(f"  {a['name']:22s} x {b['name']:22s} overlap {np.round(ov,3)}")


if __name__ == "__main__":
    main()
