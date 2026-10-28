import os

import numpy as np
import spatialgeometry as geometry


SPHERE = dict(file="unit_sphere.stl", rings=12, segments=24)


def _unit_sphere_stl(path, rings, segments):
    th = np.linspace(0.0, np.pi, rings + 1)
    ph = np.linspace(0.0, 2.0 * np.pi, segments + 1)
    v = lambda i, j: (np.sin(th[i]) * np.cos(ph[j]), np.sin(th[i]) * np.sin(ph[j]), np.cos(th[i]))
    tris = []
    for i in range(rings):
        for j in range(segments):
            a, b, c, d = v(i, j), v(i + 1, j), v(i + 1, j + 1), v(i, j + 1)
            if i > 0:
                tris.append((a, b, d))
            if i < rings - 1:
                tris.append((b, c, d))
    with open(path, "w") as f:
        f.write("solid unit_sphere\n")
        for t in tris:
            n = np.cross(np.subtract(t[1], t[0]), np.subtract(t[2], t[0]))
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(f"facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n outer loop\n")
            for p in t:
                f.write(f"  vertex {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            f.write(" endloop\nendfacet\n")
        f.write("endsolid unit_sphere\n")


def sphere_file():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), SPHERE["file"])
    if not os.path.exists(path):
        _unit_sphere_stl(path, SPHERE["rings"], SPHERE["segments"])
    return path


def ellipsoid_shape(radii, colour):
    radii = [float(r) for r in radii]
    if hasattr(geometry, "Ellipsoid"):
        return geometry.Ellipsoid(radii=radii, color=colour)
    return geometry.Mesh(sphere_file(), scale=radii, color=colour)
