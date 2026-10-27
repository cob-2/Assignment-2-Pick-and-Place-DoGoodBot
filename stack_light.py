import spatialgeometry as geometry
from spatialmath import SE3

import scene_config as cfg


STACK = dict(
    lamps=("green", "amber", "red"),
    cover=(0.032, 0.052),                  # radius, length - just larger than a lamp
    cover_colour=(0.18, 0.18, 0.20, 1.0),
    parked=SE3(0, 0, -100),
)


class StackLight:
    def __init__(self):
        specs = {s["name"]: s for s in cfg.resolved_static()}
        self.lamp = {k: SE3(*specs[f"stacklight_{k}"]["xyz"]) for k in STACK["lamps"]}
        self.state = "idle"
        self.covers = {}

    def build(self, env):
        radius, length = STACK["cover"]
        for k in STACK["lamps"]:
            shape = geometry.Cylinder(radius=radius, length=length, pose=STACK["parked"],
                                      color=STACK["cover_colour"])
            env.add(shape)
            self.covers[k] = shape
        self.show()
        return self

    def show(self):
        lit = "green" if self.state == "idle" else "red"
        for k, shape in self.covers.items():
            shape.T = (STACK["parked"] if k == lit else self.lamp[k]).A

    def idle(self):
        self.state = "idle"
        self.show()

    def running(self):
        self.state = "running"
        self.show()
