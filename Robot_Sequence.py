import argparse
from math import pi

import numpy as np
import swift
import spatialgeometry as geometry
from spatialmath import SE3

import card_vault_scene as cvs
import scene_config as cfg
import cobotta_collision_detection as cell
import vault_sequence as vault
import ur3e_outfeed as outfeed
import kawasaki_collision_detection as kcd
import ur3e_collision_detection as ucd


# ----------------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------------
ARM = "B_inspect"

TASK = dict(
    home_deg=(0, 10, 100, 0, 70, 0),
    park_deg=(-95, 15, 115, 0, 50, 0),
    tool_length=0.060,
    link_radii=(0.055, 0.050, 0.045, 0.036, 0.028, 0.022),
    grip_depth=0.015,
    lift=0.110,
    scan_height=0.080,
    steps=35,
    check_every=2,
    margin=0.005,
    jaw_margin=0.002,
    replan_tries=15,
    point_spacing=0.012,
    dt=0.03,
    pause=0.4,
)

TOOL_DOWN = SE3.Rx(pi)


# ============================================================================
# EMERGENCY STOP
# ============================================================================
class Halt(Exception):
    pass


class Restart(Exception):
    """RESTART SCENE pressed while the robots are moving."""


class StopGate:
    """Every motion step goes through here, so the panel buttons work mid-cycle."""

    def __init__(self, env, ui):
        self.env, self.ui = env, ui
        self.view = None

    def step(self, dt=0.0):
        if self.view is not None:
            self.view.update()
        self.env.step(dt)
        if self.ui.get("estop"):
            raise Halt
        if self.ui.get("request") == "restart":
            raise Restart
        if self.ui.get("request") == "start":      # already running - ignore it
            self.ui["request"] = None


def checker_cfg():
    g = cell.CONFIG["gripper"]
    return dict(
        robot=dict(link_radii=TASK["link_radii"], tool_length=TASK["tool_length"],
                   link_boxes=cell.CONFIG["robot"]["link_boxes"],
                   home_deg=TASK["home_deg"]),
        motion=dict(steps=TASK["steps"], check_every=TASK["check_every"],
                    margin=TASK["margin"], jaw_margin=TASK["jaw_margin"],
                    replan_tries=TASK["replan_tries"], dt=TASK["dt"],
                    point_spacing=TASK["point_spacing"]),
        gripper=dict(open=g["open"], closed=g["closed"], finger=g["finger"],
                     pad=g["pad"], pad_xyz=g["pad_xyz"], clearance=g["clearance"]),
    )


# ----------------------------------------------------------------------------
# COLLISION VIEW
# ----------------------------------------------------------------------------
VIEW = dict(box=(0.20, 0.85, 0.95, 0.22), ellipsoid=(1.00, 0.45, 0.10, 0.30),
             pad=0.120, samples=300, parked=SE3(0, 0, -100).A)


class CollisionView:
    def __init__(self, env, task):
        self.env, self.task = env, task
        self.on = False
        self.boxes = []
        self.arms = []

    def sources(self):
        out = [(self.task.robot, lambda r: cell.arm_ellipsoids(r, self.task.cfg, r.q))]
        if self.task.vault is not None:
            out.append((self.task.vault.robot, lambda r: kcd.arm_ellipsoids(r, r.q)))
        if self.task.outfeed is not None:
            out.append((self.task.outfeed.robot, lambda r: ucd.arm_ellipsoids(r, r.q)))
        return out

    def envelope(self):
        rng = np.random.default_rng(0)
        out = []
        for robot, _ in self.sources():
            pts = []
            for _ in range(VIEW["samples"]):
                q = np.array([rng.uniform(*l.qlim) for l in robot.links])
                pts += [T.t for T in robot.fkine_all(q)]
            out.append(np.asarray(pts, dtype=float))
        return out

    def in_reach(self, ob, envelope):
        for pts in envelope:
            rel = np.abs((pts - ob.pose.t) @ ob.pose.R) - ob.half
            d = np.linalg.norm(np.clip(rel, 0.0, None), axis=1)
            if float(d.min()) <= VIEW["pad"]:
                return True
        return False

    def build(self):
        envelope = self.envelope()
        for ob in self.task.world.obstacles:
            if not self.in_reach(ob, envelope):
                continue
            shape = geometry.Cuboid(scale=list(np.asarray(ob.half) * 2), pose=ob.pose,
                                    color=VIEW["box"])
            self.env.add(shape)
            self.boxes.append((shape, ob))
        for robot, fn in self.sources():
            shapes = []
            for item in fn(robot):
                radii = np.asarray(item[-1], dtype=float)
                if hasattr(geometry, "Ellipsoid"):
                    shape = geometry.Ellipsoid(radii=list(radii), color=VIEW["ellipsoid"])
                else:                    # older spatialgeometry: the box around the ellipsoid
                    shape = geometry.Cuboid(scale=list(radii * 2), color=VIEW["ellipsoid"])
                self.env.add(shape)
                shapes.append(shape)
            self.arms.append((robot, fn, shapes))
        self.place()

    def alpha(self, shape, value):
        if not self.task.headless:
            shape.set_alpha(value)

    def place(self):
        # Swift only re-sends a shape's pose after it is added, not its colour, so a
        # hidden shape is also parked out of sight rather than relying on opacity alone.
        for shape, ob in self.boxes:
            shown = self.on and ob.active
            shape.T = ob.pose.A if shown else VIEW["parked"]
            self.alpha(shape, VIEW["box"][3] if shown else 0.0)
        for robot, fn, shapes in self.arms:
            for shape, item in zip(shapes, fn(robot)):
                centre, R, _ = item[-3:]
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = centre
                shape.T = T if self.on else VIEW["parked"]
                self.alpha(shape, VIEW["ellipsoid"][3] if self.on else 0.0)

    def update(self):
        if self.on:
            self.place()

    def toggle(self):
        self.on = not self.on
        if self.on and not self.boxes:
            self.build()
        else:
            self.place()
        self.env.step(0.0)
        return self.on


# ----------------------------------------------------------------------------
# THE ARM'S HAND
# ----------------------------------------------------------------------------
class Hand:

    def __init__(self, robot):
        self.robot = robot

    @property
    def opening(self):
        return self.robot.gripper.half

    @opening.setter
    def opening(self, half):
        self.robot.gripper.set_opening(2 * float(half))

    def update(self, _T=None):
        self.robot.gripper.update(self.robot.flange_pose())


# ----------------------------------------------------------------------------
# WORLD
# ----------------------------------------------------------------------------
def spec_box(spec):
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


def build_world(base_z, spacing):
    world = cell.World(spacing)
    for spec in cfg.resolved_static() + cfg.card_objects():
        if spec["group"] == "markers" and not cfg.SHOW_MARKERS:
            continue
        size, pose = spec_box(spec)
        if size is None:
            continue
        top = pose.t[2] + size[2] / 2
        under_arm = top <= base_z + 0.012
        world.add_box(spec["name"], size, pose, (0, 1, 2) if under_arm else ())
    return world


# ----------------------------------------------------------------------------
# POSES
# ----------------------------------------------------------------------------
def over(xy, z, yaw=0.0):
    return SE3(xy[0], xy[1], z + TASK["tool_length"]) * TOOL_DOWN * SE3.Rz(yaw)


def grasp(xy, top_z, depth=None):
    return over(xy, top_z - (TASK["grip_depth"] if depth is None else depth))


def approach(xy, top_z, depth=None, lift=None):
    return SE3(0, 0, TASK["lift"] if lift is None else lift) * grasp(xy, top_z, depth)


def spot_poses(spot):
    dx, dy = spot.get("offset", (0.0, 0.0))
    xy = (spot["xy"][0] + dx, spot["xy"][1] + dy)
    g = grasp(xy, spot["top"], spot.get("depth"))
    return g, approach(xy, spot["top"], spot.get("depth"), spot.get("lift"))


# ----------------------------------------------------------------------------
# THE CELL
# ----------------------------------------------------------------------------
class   PackagingTask:
    def __init__(self):
        self.env = swift.Swift()
        self.cfg = checker_cfg()

    def build(self, headless=False):
        self.headless = headless
        self.face = None
        self.ui = dict(face="front", go=False, reject=False, estop=False,
                       request=None, card=None)
        self.status = None
        self.env.launch(realtime=not headless, headless=headless)
        self.shapes, robots = cvs.build_scene(self.env)
        self.robot = robots[ARM]
        if self.robot is None:
            raise RuntimeError(f"{ARM} failed to load - see the placeholder message")

        self.robot.tool = SE3()
        self.robot.q = np.deg2rad(TASK["home_deg"])

        self.world = build_world(float(self.robot.base.t[2]), TASK["point_spacing"])
        self.hand = Hand(self.robot)
        self.gate = StopGate(self.env, self.ui)
        self.ctrl = cell.Controller(self.robot, self.hand, self.gate, self.world, self.cfg)
        self.start_poses = {n: SE3(sh.T) for n, sh in self.shapes.items()}
        self.ctrl.refresh()
        self.slab_card = None
        self.packed = False
        kawasaki = robots.get(vault.ARM)
        self.vault = (vault.VaultArm(self.env, kawasaki, self.shapes, self.gate,
                                     cobotta=self.robot, cobotta_cfg=self.cfg, say=self.say)
                      if kawasaki is not None else None)
        ur3e = robots.get(outfeed.ARM)
        self.outfeed = (outfeed.OutfeedArm(self.env, ur3e, self.shapes, self.gate, say=self.say,
                                           others=[("cobotta", self.robot, TASK["link_radii"],
                                                    TASK["tool_length"])])
                        if ur3e is not None else None)
        self.view = CollisionView(self.env, self)
        self.gate.view = self.view
        if not headless:
            self.add_panel()
        self.env.step(0.0)
        return self

    def spawn_slab(self):
        size = (cfg.SLAB["w"], cfg.SLAB["t"], cfg.SLAB["h"])
        x, y = cfg.TRANSFER["x"], cfg.TRANSFER["y"]
        z = cfg.BENCH_H + cfg.RACK["base_h"] + size[2] / 2
        pose = SE3(x, y, z)
        shape = geometry.Cuboid(scale=list(size), pose=pose, color=cfg.C["card"])
        self.env.add(shape)
        self.shapes["slab"] = shape
        self.world.add_box("slab", size, pose)
        self.start_poses["slab"] = pose
        self.env.step(0.0)
        print("   (slab placed in the hand-off holder - stands in for Arm A)")
        return shape

    def sync_world(self):
        """Copy every object's pose from Swift into the COBOTTA's world (the Kawasaki moves cards)."""
        held = [ob.name for _, ob, _ in self.ctrl.carried]
        for ob in self.world.obstacles:
            shape = self.shapes.get(ob.name)
            if shape is not None and ob.name not in held:
                ob.pose = SE3(np.array(shape.T), check=False)

    def hand_over(self, card):
        """The card the Kawasaki just delivered becomes the COBOTTA's "slab"."""
        self.world.get(card).name = "slab"
        self.shapes["slab"] = self.shapes[card]
        self.slab_card = card
        self.sync_world()

    def take_back(self):
        """Undo hand_over: the slab goes back to being card_cXrY."""
        if self.slab_card is None:
            return None
        card, self.slab_card = self.slab_card, None
        try:
            self.world.get("slab").name = card
        except StopIteration:
            pass
        self.shapes.pop("slab", None)
        return card

    def chosen_card(self):
        avail = self.vault.available()
        card = self.ui.get("card")
        if card is None and avail:
            card = avail[0]
        return card if card in avail else None

    def spots(self):
        p, m = cfg.PACK, cfg.MOUNT
        slab_h = cfg.SLAB["h"]
        mount_z = m["at"]["inspection"][2] + m["shelf"][2]
        return dict(
            handoff=dict(xy=(cfg.TRANSFER["x"], cfg.TRANSFER["y"]),
                         top=cfg.BENCH_H + cfg.RACK["base_h"] + slab_h,
                         lift=0.070),
            scan=dict(xy=m["at"]["inspection"][:2],
                      top=mount_z + TASK["scan_height"] + slab_h,
                      held=True),
            mount=dict(xy=m["at"]["inspection"][:2], top=mount_z + slab_h),
            giveback=dict(xy=m["at"]["return"][:2],
                          top=m["at"]["return"][2] + m["shelf"][2] + slab_h),
            holder_drop=dict(xy=p["holder_xy"],
                             top=cfg.BENCH_H + p["wall"] + p["drop"] + slab_h),
            box=dict(xy=p["holder_xy"], top=cfg.BENCH_H + p["outer"][2],
                     depth=p["grip_below_rim"]),
            outfeed=dict(xy=p["outfeed_xy"],
                         top=cfg.BENCH_H + p["pad"][2] + p["outer"][2],
                         depth=p["grip_below_rim"], lift=0.075),
        )

    def near(self, xy, top, reach=0.075, band=0.25):
        held = [ob.name for _, ob, _ in self.ctrl.carried]
        out = []
        for ob in self.world.obstacles:
            if ob.name in held:
                continue
            d = np.abs(np.asarray(xy) - ob.pose.t[:2]) - (ob.half[:2] + reach)
            lo, hi = ob.pose.t[2] - ob.half[2], ob.pose.t[2] + ob.half[2]
            if np.all(d < 0.0) and lo <= top + 0.05 and hi >= top - band:
                out.append(ob.name)
        return out

    def riders(self, name, window=0.30, tol=0.004):
        host = self.world.get(name)
        floor = host.pose.t[2] - host.half[2]
        held = [ob.name for _, ob, _ in self.ctrl.carried]
        out = []
        for ob in self.world.obstacles:
            if ob is host or ob.name in held:
                continue
            reach = np.abs(ob.pose.t[:2] - host.pose.t[:2]) + ob.half[:2]
            inside = np.all(reach <= host.half[:2] + tol)
            base = ob.pose.t[2] - ob.half[2]
            if inside and floor - tol <= base <= floor + window:
                out.append(ob.name)
        return out

    def jaw_links(self):
        return (len(self.cfg["robot"]["link_boxes"]),)

    def move_linear(self, T_goal, steps=12):
        ctrl = self.ctrl
        T0 = self.robot.fkine(self.robot.q)
        qs, q = [], self.robot.q.copy()
        for a in np.linspace(0.0, 1.0, steps)[1:]:
            T = SE3.Rt(T_goal.R, T0.t + a * (T_goal.t - T0.t))
            sol = self.robot.ikine_LM(T, q0=q, joint_limits=True, slimit=40, ilimit=200)
            if not sol.success:
                return ctrl.move_to_pose(T_goal)
            q = sol.q
            qs.append(q)
        start = cell.contacts(self.robot, self.cfg, self.world, self.robot.q)
        k, hit = cell.path_collision(self.robot, self.cfg, self.world, np.array(qs), start)
        if hit is not None:
            print(f"    straight move blocked at step {k} ({hit}) - planning around")
            return ctrl.move_to_pose(T_goal)
        for step_q in qs:
            self.robot.q = step_q
            ctrl.refresh()
            self.gate.step(TASK["dt"])
        return True

    def pick(self, name, spot, grip=None, width=None):
        ctrl, s = self.ctrl, spot
        grip = grip or name
        w = (float(self.world.get(grip).half[1] * 2) if width is None else float(width))
        g, a = spot_poses(s)
        ok = ctrl.move_to_pose(a)
        rides = self.riders(name)
        own = list(dict.fromkeys([name, grip] + rides))
        nearby = [n for n in self.near(s["xy"], s["top"]) if n not in own]
        with ctrl.ignoring(own), ctrl.ignoring(nearby, links=self.jaw_links()):
            ok &= self.move_linear(g)
            ctrl.grasp(self.shapes[name], self.world.get(name), w)
            for r in rides:
                ctrl.grasp(self.shapes[r], self.world.get(r))
            ok &= self.move_linear(a)
        return ok

    def place(self, spot, resting):
        ctrl, s = self.ctrl, spot
        names = [ob.name for _, ob, _ in ctrl.carried]
        g, a = spot_poses(s)
        ok = ctrl.move_to_pose(a)
        nearby = [n for n in self.near(s["xy"], s["top"]) if n not in names]
        with ctrl.ignoring(names), ctrl.ignoring(nearby, links=self.jaw_links()):
            ok &= self.move_linear(g)
            ctrl.release(resting)
            ok &= self.move_linear(a)
        return ok

    def report_scan(self, face):
        slab = self.world.get("slab")
        lens = np.asarray(cfg.FRAMES["camera"], dtype=float)
        d = slab.pose.t - lens
        off = np.hypot(d[0], d[2])
        print(f"   camera: {face} face at {np.round(slab.pose.t, 3)}, "
              f"standoff {d[1]:.3f} m, off axis {off * 1000:.0f} mm")

    def add_panel(self):
        self.ui.update(face="front", go=False, reject=False)

        def flip(_):
            self.ui["face"] = "back" if self.ui["face"] == "front" else "front"

        def finish(reject):
            def cb(_):
                self.ui["reject"] = reject
                self.ui["go"] = True
            return cb

        def estop(_):
            self.ui["estop"] = True
            self.ui["request"] = None

        def reset(_):
            if self.ui["estop"]:
                self.ui["estop"] = False
                self.say("e-stop released - Choose a slot and press START")
            else:
                self.say("Nothing to reset")

        def request(what):
            def cb(_):
                self.ui["request"] = what
            return cb

        def view_toggle(_):
            self.say("collision boxes on" if self.view.toggle() else "collision boxes off")

        self.status = swift.Label(desc="Starting up")
        self.env.add(self.status)
        if self.vault is not None:
            names = list(self.vault.slot_of)
            labels = [f"Rack {r + 1} - Slot {c + 1}" for c, r in self.vault.slot_of.values()]
            self.ui["card"] = names[0]

            def choose(index):
                self.ui["card"] = names[int(index)]
                self.say(f"Selected {labels[int(index)]} - Choose a card and press START")

            self.env.add(swift.Select(choose, desc="Card", options=labels, value=0))
            self.env.add(swift.Button(request("start"), desc="START"))
        self.env.add(swift.Button(flip, desc="FLIP"))
        self.env.add(swift.Button(finish(False), desc="ACCEPT"))
        self.env.add(swift.Button(finish(True), desc="REJECT"))
        self.env.add(swift.Button(estop, desc="EMERGENCY STOP"))
        self.env.add(swift.Button(reset, desc="E-STOP RELEASE"))
        self.env.add(swift.Button(request("restart"), desc="RESTART SCENE"))
        self.env.add(swift.Button(view_toggle, desc="COLLISION BOXES"))

    def say(self, text):
        print(f"   {text}")
        if getattr(self, "status", None) is not None:
            self.status.desc = text

    def show_face(self, face):
        s = self.spots()["scan"]
        yaw = 0.0 if face == "front" else pi
        self.ctrl.move_to_pose(over(s["xy"], s["top"] - TASK["grip_depth"], yaw))
        self.face = face
        self.report_scan(face)

    def scan_station(self):
        self.show_face("front")
        if self.headless:
            self.show_face("back")
            return False
        self.say("Front face at the camera - FLIP to see the other side, then ACCEPT or REJECT")
        while not self.ui["go"]:
            self.gate.step(0.05)
            if self.ui["face"] != self.face:
                self.show_face(self.ui["face"])
                self.say(f"{self.face} Face at the camera - FLIP again, or ACCEPT or REJECT")
        return self.ui["reject"]

    def run(self, reject=False, card=None):
        ctrl, sp = self.ctrl, self.spots()
        home = np.deg2rad(TASK["home_deg"])
        p = cfg.PACK

        if self.vault is not None:
            if self.packed:
                self.say("the box has already gone - press Restart scene first")
                return
            card = card if card is not None else self.chosen_card()
            if card is None:
                self.say("that card is no longer in the vault - choose another")
                return
            print(f"0. Kawasaki fetches {card} into the hand-off holder")
            if not self.vault.fetch(card):
                self.say("Kawasaki could not fetch the card - holding")
                return
            self.hand_over(card)

        print("1. collect the slab from the hand-off holder")
        self.pick("slab", sp["handoff"])

        print("2. hold it at the camera and wait for the operator")
        reject = self.scan_station() or reject

        if reject:
            print("3. not wanted - stand it on the Return mount for Arm A")
            self.place(sp["giveback"], SE3(*cfg.mount_slab_xyz("return")))
            self.say("slab on the Return mount - waiting")
            self.park()
            card = self.take_back()
            if self.vault is not None and card is not None:
                print(f"4. Kawasaki puts {card} back in its slot")
                self.vault.sync()
                self.vault.put_back(card)
                self.say(f"{card} is back in the vault - choose a card and press START")
            print(f"done - {ctrl.replans} re-plan(s), {ctrl.faults} fault(s)")
            return

        print("3. stand the slab in the packaging holder")
        self.place(sp["holder_drop"], SE3(*cfg.holder_slab_xyz()))

        print("4. set the finished box on the outfeed pad")
        self.pick("holder_base", sp["box"], grip="holder_wall_py",
                  width=p["outer"][1])
        rest = SE3(p["outfeed_xy"][0], p["outfeed_xy"][1],
                   cfg.BENCH_H + p["pad"][2] + p["wall"] / 2)
        self.place(sp["outfeed"], rest)

        print("5. back to the waiting pose, clear of the UR3e")
        self.park()
        self.take_back()                     # the card stays in the box; its slot stays empty
        self.packed = True

        if self.outfeed is not None:         # steps 6-8: UR3e puts the box on the conveyor
            self.outfeed.deliver()
        print(f"done - {ctrl.replans} re-plan(s), {ctrl.faults} fault(s)")

    def park(self):
        ok = self.ctrl.move_to(np.deg2rad(TASK["park_deg"]))
        self.say("Waiting - clear of the UR3e's path")
        return ok

    def reset_scene(self):
        self.take_back()
        self.packed = False
        self.ctrl.carried.clear()
        for name, pose in self.start_poses.items():
            self.shapes[name].T = pose.A
            try:
                ob = self.world.get(name)
            except StopIteration:
                continue
            ob.pose = pose
            ob.active = True
        self.robot.gripper.set_opening(2 * cell.CONFIG["gripper"]["open"])
        self.robot.q = np.deg2rad(TASK["home_deg"])
        self.ctrl.replans = 0
        self.ctrl.faults = 0
        self.ui.update(face="front", go=False, reject=False)
        self.face = None
        if self.vault is not None:
            self.vault.reset()
        if self.outfeed is not None:
            self.outfeed.reset()
        self.ctrl.refresh()
        self.env.step(0.0)

    def console(self):
        self.say("Ready - choose a card and press START")
        while True:
            self.env.step(0.05)
            req = self.ui.get("request")
            if req is None:
                continue
            self.ui["request"] = None
            if self.ui["estop"]:
                self.say("E-STOP latched - press E-STOP RELEASE")
                continue
            if req == "start":
                self.ui.update(go=False, reject=False, face="front")
                try:
                    self.run()
                except Halt:
                    self.say("E-STOP - motion stopped. Press E-STOP RELEASE, then RESTART SCENE")
                except Restart:
                    self.restart()
            elif req == "restart":
                self.restart()

    def restart(self):
        self.ui["request"] = None
        self.reset_scene()
        self.say("Scene restarted - choose a card and press START")

    def park_clearance(self, width=0.10, heights=(0.04, 0.12, 0.22, 0.32), n=40):
        q_park = np.deg2rad(TASK["park_deg"])
        ur = cfg.ROBOTS[outfeed.ARM]["base_xyz"]
        pad = cfg.PACK["outfeed_xy"]
        a = np.array([ur[0], ur[1]])
        b = np.array([pad[0], pad[1]])
        pts = []
        for t in np.linspace(0.0, 1.0, n):
            xy = a + t * (b - a)
            for off in (-width / 2, 0.0, width / 2):
                perp = np.array([-(b - a)[1], (b - a)[0]])
                perp = perp / (np.linalg.norm(perp) or 1.0)
                for h in heights:
                    pts.append([*(xy + off * perp), cfg.BENCH_H + h])
        pts = np.asarray(pts)
        worst = np.inf
        for idx, centre, R, radii in cell.arm_ellipsoids(self.robot, self.cfg, q_park):
            rel = (pts - centre) @ R
            d = np.sqrt(np.sum((rel / radii) ** 2, axis=1))
            worst = min(worst, float(d.min()))
        return worst

    def reach_report(self):
        print(f"{'spot':14s} {'fingertip xyz':28s} {'reach':>7s}  IK")
        base = self.robot.base.t
        for name, s in self.spots().items():
            g, a = spot_poses(s)
            poses = [("grasp", g)] if s.get("held") else [("grasp", g), ("approach", a)]
            for tag, T in poses:
                tip = T.t - np.array([0, 0, TASK["tool_length"]])
                d = float(np.linalg.norm(tip - base))
                sol = self.robot.ikine_LM(T, q0=np.deg2rad(TASK["home_deg"]),
                                          joint_limits=True, slimit=40, ilimit=200)
                err = (float(np.linalg.norm(self.robot.fkine(sol.q).t - T.t))
                       if sol.success else float("nan"))
                flag = "OK " if sol.success and err < 2e-3 else "FAIL"
                print(f"{name + '/' + tag:22s} {np.round(tip, 3)!s:28s} "
                      f"{d:6.3f}  {flag} {err * 1000:5.2f} mm")
        c = self.park_clearance()
        print(f"\nwaiting pose vs the UR3e's corridor to the pad: "
              f"{'clear' if c > 1.0 else 'BLOCKED'} (worst normalised distance {c:.2f}, "
              f"1.0 = touching a link ellipsoid)")
        hit = cell.first_collision(self.robot, self.cfg, self.world,
                                   np.deg2rad(TASK["park_deg"]),
                                   cell.contacts(self.robot, self.cfg, self.world,
                                                 np.deg2rad(TASK["home_deg"])))
        print(f"waiting pose collisions: {hit or 'none'}")
        if self.outfeed is not None:
            print()
            self.outfeed.reach_report()


def main():
    ap = argparse.ArgumentParser(description="COBOTTA scan and packaging task")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--reject", action="store_true", help="fail the scan")
    ap.add_argument("--reach", action="store_true", help="IK check every pose only")
    ap.add_argument("--card", default=None, help="card for --headless, e.g. card_c1r2")
    args = ap.parse_args()

    task = PackagingTask().build(headless=args.headless or args.reach)
    if task.vault is None or args.reach:
        task.spawn_slab()                 # no Kawasaki: stand-in slab in the holder
    if args.reach:
        task.reach_report()
        return
    if args.headless:
        task.run(reject=args.reject, card=args.card)
        return
    task.console()


if __name__ == "__main__":
    main()
