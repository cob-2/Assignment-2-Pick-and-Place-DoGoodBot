from contextlib import contextmanager
from math import pi

import numpy as np
from spatialmath import SE3

import scene_config as cfg
import ur3e_collision_detection as ucd


# ----------------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------------
ARM = ucd.ARM

TASK = dict(
    tcp=0.222,              # flange to RG2 grip point (m); only used if the model has no RG2
    open_gap=0.060,         # RG2 jaw gap when approaching the box (m)
    clearance=0.001,        # each side, added to the box width when closing on it
    grip_below_rim=0.030,   # pads this far below the box rim (the COBOTTA uses 20 mm; a
                            # little deeper keeps the bulkier RG2 body clear of the box)
    pick_lift=0.065,        # approach height above the grasp at the pad
    drop_lift=0.065,        # and above the drop on the spur
    min_lift=0.035,         # lowest approach that still keeps the pads above the box rim
    transit_z=1.15,         # TCP height while carrying the box round to the belt (higher
                            # stretches the arm out to its elbow singularity)
    transit_r=0.20,         # and at least this far from the UR3e base axis (stays clear of
                            # the shoulder singularity a straight line would pass through)
    slow=0.10,              # RMRC TCP speed near the box (m/s)
    fast=0.25,              # RMRC TCP speed in transit (m/s)
    qd_max=1.5,             # joint speed limit (rad/s); faster steps are scaled down
    manip_min=0.010,        # below this manipulability, damping switches on
    damping_max=0.05,       # largest damping factor (lambda) at a singularity
    rmrc_tol=0.002,         # final TCP position error accepted (m)
    steps=40,
    replan_tries=20,
    dt=0.03,
    belt_speed=0.25,        # m/s along each belt
    belt_stop_x=0.50,      # where the box stops on the main belt (outfeed end)
)

TOOL_DOWN = SE3.Rx(pi)


class NoGate:
    """Stands in for the StopGate when there is no Swift window."""
    def step(self, dt=0.0):
        pass


# ----------------------------------------------------------------------------
# POSES
# ----------------------------------------------------------------------------
def box_top(floor_z):
    return floor_z + cfg.PACK["outer"][2]


def spots():
    p = cfg.PACK
    belt_top = cfg.CONV_TOP + 0.012
    return dict(
        pad=dict(xy=p["outfeed_xy"], floor=cfg.BENCH_H + p["pad"][2], lift=TASK["pick_lift"]),
        spur=dict(xy=cfg.SPUR_DROP_XY, floor=belt_top, lift=TASK["drop_lift"]),
    )


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class OutfeedArm:
    """The UR3e's part of the cycle. shapes: name -> Swift shape (shared with the COBOTTA task)."""

    def __init__(self, env, robot, shapes, gate, say=print, others=()):
        """others: (name, robot, link_radii, tool_length) for arms the UR3e must keep
        clear of (the parked COBOTTA); their links are re-read on every sync()."""
        self.env, self.robot, self.shapes, self.gate, self.say = env, robot, shapes, gate, say
        self.others = list(others)
        # fkine() is the bare flange (as for the COBOTTA) and the TCP is added here.
        # The RG2's grip point drops as the jaws close, so plan with the grip point at
        # the gap that holds the box: the pads then close onto the planned height.
        self.robot.tool = SE3()
        self.tcp = self.grip_tcp()
        self.home = np.array(cfg.ROBOTS[ARM]["q0"], dtype=float)
        self.robot.q = self.home.copy()
        self.world = ucd.world_from_scene(cfg)
        self.sync_others()
        self.opening = TASK["open_gap"]
        self.set_jaws(TASK["open_gap"])
        self.lifts = self.fit_lifts()
        self.parts = []         # (shape name, T_flange_part) for the box parts being carried
        self.load = []          # (size, T_flange_box) for the collision checker
        self.box = []           # names of the box parts on the spur / belt
        self.faults = 0
        self.replans = 0

    # -- bookkeeping ----------------------------------------------------------
    def held(self):
        return [name for name, _ in self.parts]

    def sync(self):
        """Copy every object's pose from Swift (the COBOTTA and Kawasaki move things)."""
        names = set(self.world.names())
        for name, shape in self.shapes.items():
            if name in self.held():
                continue
            if name not in names and hasattr(shape, "scale"):
                self.world.add_box(name, tuple(shape.scale), SE3(np.array(shape.T), check=False))
                names.add(name)
        for ob in self.world.obstacles:
            shape = self.shapes.get(ob.name)
            if shape is not None and ob.name not in self.held():
                ob.pose = SE3(np.array(shape.T), check=False)
        self.sync_others()

    def sync_others(self):
        """Each link of another arm becomes a box from joint to joint, as thick as its
        link radius, plus one for its hand, posed where the arm is now."""
        for name, arm, radii, tool in self.others:
            frames = [np.asarray(T.A if hasattr(T, "A") else T) for T in arm.fkine_all(arm.q)]
            pts = [F[:3, 3] for F in frames]
            pts.append((frames[-1] @ [0, 0, tool, 1])[:3])
            for i in range(len(pts) - 1):
                a, b = pts[i], pts[i + 1]
                d = b - a
                length = float(np.linalg.norm(d))
                if length < 1e-6:
                    continue
                z = d / length
                x = np.cross([1.0, 0, 0] if abs(z[0]) < 0.9 else [0, 1.0, 0], z)
                x /= np.linalg.norm(x)
                pose = SE3.Rt(np.column_stack((x, np.cross(z, x), z)), (a + b) / 2)
                r = radii[min(i, len(radii) - 1)]
                key = f"{name}_link{i}"
                if key in self.world.names():
                    ob = self.world.get(key)
                    ob.pose = pose
                    if np.allclose(ob.half, [r, r, length / 2 + r]):
                        continue
                    self.world.obstacles.remove(ob)
                self.world.add_box(key, (2 * r, 2 * r, length + 2 * r), pose)

    def riders(self, name, window=0.30, tol=0.004):
        host = self.world.get(name)
        floor = host.pose.t[2] - host.half[2]
        out = []
        for ob in self.world.obstacles:
            if ob is host or ob.name not in self.shapes:
                continue
            reach = np.abs(ob.pose.t[:2] - host.pose.t[:2]) + ob.half[:2]
            base = ob.pose.t[2] - ob.half[2]
            if np.all(reach <= host.half[:2] + tol) and floor - tol <= base <= floor + window:
                out.append(ob.name)
        return out

    def near(self, xy, top, reach=0.08, band=0.25):
        out = []
        for ob in self.world.obstacles:
            if ob.name in self.held():
                continue
            d = np.abs(np.asarray(xy) - ob.pose.t[:2]) - (ob.half[:2] + reach)
            lo, hi = ob.pose.t[2] - ob.half[2], ob.pose.t[2] + ob.half[2]
            if np.all(d < 0.0) and lo <= top + 0.05 and hi >= top - band:
                out.append(ob.name)
        return out

    @contextmanager
    def ignoring(self, names):
        """world.ignoring, but whatever is in the hand when the block ends stays
        switched off (it may have been picked up inside the block)."""
        with self.world.ignoring([n for n in names if n not in self.held()]):
            yield
        for name in self.held():
            self.world.get(name).active = False

    # -- the RG2 --------------------------------------------------------------
    def grip_gap(self):
        return cfg.PACK["outer"][1] + 2 * TASK["clearance"]

    def grip_tcp(self):
        g = getattr(self.robot, "gripper", None)
        if g is None or not hasattr(g, "set_opening") or not hasattr(g, "tcp"):
            return SE3(0, 0, TASK["tcp"])
        g.set_opening(self.grip_gap())
        tcp = SE3(g.tcp)
        g.set_opening(TASK["open_gap"])
        return tcp

    def set_jaws(self, gap):
        self.opening = float(gap)
        g = getattr(self.robot, "gripper", None)
        if g is not None and hasattr(g, "set_opening"):
            g.set_opening(self.opening)
            if hasattr(self.robot, "flange_pose"):
                g.update(self.robot.flange_pose())

    def move_jaws(self, gap, steps=12):
        for w in np.linspace(self.opening, gap, steps)[1:]:
            self.set_jaws(w)
            self.gate.step(TASK["dt"])

    # -- kinematics -----------------------------------------------------------
    def tcp_pose(self, q):
        return self.robot.fkine(q) * self.tcp

    def tcp_jacobian(self, q):
        """6xn base-frame Jacobian of the RG2 grip point: the flange Jacobian with the
        linear rows shifted by w x r, r = flange -> grip point in world axes."""
        J = np.asarray(self.robot.jacob0(q))
        r = self.robot.fkine(q).R @ self.tcp.t
        shift = np.eye(6)
        shift[:3, 3:] = -skew(r)
        return shift @ J

    def solve(self, T_tcp, q0=None, tries=4):
        """IK for a TCP pose. ikine_LM restarts from random points, so it can land in
        any arm configuration; each answer is wrapped to the turn nearest the current
        joints and the closest one wins, so the arm doesn't swing round the long way."""
        T = T_tcp * self.tcp.inv()
        ref = np.asarray(self.robot.q if q0 is None else q0, dtype=float)
        lim = np.asarray(self.robot.qlim)
        best, best_cost, found = None, np.inf, 0
        for s in [ref, self.home] * tries:
            sol = self.robot.ikine_LM(T, q0=s, joint_limits=True, ilimit=300, slimit=60)
            if not sol.success:
                continue
            q = ref + (np.asarray(sol.q) - ref + pi) % (2 * pi) - pi
            if np.any(q < lim[0]) or np.any(q > lim[1]):
                q = np.asarray(sol.q, dtype=float)
            if np.linalg.norm(self.robot.fkine(q).t - T.t) > 1e-3:
                continue
            cost = float(np.max(np.abs(q - ref)))
            if cost < best_cost:
                best, best_cost = q, cost
            found += 1
            if found >= tries:
                break
        return best

    def rmrc(self, T_goal, speed, q_start=None, via=()):
        """Resolved-rate motion control of the TCP to T_goal: along a straight line, or
        along the polyline through the 'via' points when given.

        The TCP moves along the path with a quintic (smooth start and stop) profile.
        Each step: xd = [v; w] from the gap between where the TCP is and where it should
        be next step, qd = J^T (J J^T + lambda^2 I)^-1 xd, with lambda > 0 only near a
        singularity (manipulability below manip_min), then qd is capped at qd_max and
        integrated: q += qd dt. Returns (qs, report) or (None, why)."""
        dt = TASK["dt"]
        q = np.asarray(self.robot.q if q_start is None else q_start, dtype=float).copy()
        T0 = self.tcp_pose(q)
        pts = np.array([T0.t] + [np.asarray(v, dtype=float) for v in via] + [T_goal.t])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.r_[0.0, np.cumsum(seg)] / max(seg.sum(), 1e-9)
        ang = float(abs((T0.inv() * T_goal).angvec()[0]))
        n = int(np.ceil(max(seg.sum() / speed, ang / 1.0, 0.4) / dt))
        lim = np.asarray(self.robot.qlim)
        qs, worst_qd, min_m, err = [], 0.0, np.inf, np.inf
        for k in range(1, n + 15):                       # a few extra steps to settle
            tau = min(k / n, 1.0)
            s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
            p_des = np.array([np.interp(s, cum, pts[:, i]) for i in range(3)])
            R_des = T0.interp(T_goal, s).R
            T_now = self.tcp_pose(q)
            v = (p_des - T_now.t) / dt
            Re = R_des @ T_now.R.T
            w = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) / dt
            J = self.tcp_jacobian(q)
            m = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
            min_m = min(min_m, m)
            lam = (TASK["damping_max"] * (1 - (m / TASK["manip_min"]) ** 2)
                   if m < TASK["manip_min"] else 0.0)
            qd = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), np.r_[v, w])
            peak = float(np.max(np.abs(qd)))
            if peak > TASK["qd_max"]:
                qd *= TASK["qd_max"] / peak
            worst_qd = max(worst_qd, min(peak, TASK["qd_max"]))
            q = q + qd * dt
            if np.any(q < lim[0]) or np.any(q > lim[1]):
                return None, "joint limit"
            qs.append(q.copy())
            err = float(np.linalg.norm(self.tcp_pose(q).t - T_goal.t))
            if k >= n and err < TASK["rmrc_tol"] / 4:
                break
        if err > TASK["rmrc_tol"]:
            return None, f"ended {err * 1000:.1f} mm off"
        return np.array(qs), (f"{len(qs)} steps, peak joint speed {worst_qd:.2f} rad/s, "
                              f"min manipulability {min_m:.3f}, error {err * 1000:.2f} mm")

    def arc_via(self, T_from, T_to, z, r_min, n=8):
        """Points on an arc round the base axis from above T_from to above T_to, at
        height z, the radius blending from the start's to the goal's but never under
        r_min: a straight line between the pad and the belt passes close to the base
        axis, where the UR3e is singular."""
        b = self.robot.base.t[:2]
        d0, d1 = T_from.t[:2] - b, T_to.t[:2] - b
        a0, a1 = np.arctan2(d0[1], d0[0]), np.arctan2(d1[1], d1[0])
        r0, r1 = np.linalg.norm(d0), np.linalg.norm(d1)
        da = (a1 - a0 + pi) % (2 * pi) - pi           # the short way round
        out = []
        for t in np.linspace(0.0, 1.0, n)[1:-1]:
            r = max(r_min, r0 + t * (r1 - r0))
            out.append((b[0] + r * np.cos(a0 + t * da), b[1] + r * np.sin(a0 + t * da), z))
        return out

    # -- checked motion -------------------------------------------------------
    def clear(self, qs):
        start = ucd.contacts(self.robot, self.world, self.robot.q, self.load)
        k, hit = ucd.path_collision(self.robot, self.world, qs, start, self.load)
        return None if hit is None else f"{hit} at step {k}"

    def run(self, qs):
        for q in qs:
            self.robot.q = q
            self.refresh()
            self.gate.step(TASK["dt"])

    def refresh(self):
        if hasattr(self.robot, "_update_3dmodel"):
            self.robot._update_3dmodel()
        g = getattr(self.robot, "gripper", None)
        if g is not None and hasattr(self.robot, "flange_pose"):
            g.update(self.robot.flange_pose())
        T = self.robot.fkine(self.robot.q)
        for name, grip in self.parts:
            W = T * grip
            self.shapes[name].T = W.A
            self.world.get(name).pose = W

    def straight(self, T_goal, speed, label, via=()):
        """RMRC to T_goal; if the path is blocked, fall back to a planned joint move."""
        qs, report = self.rmrc(T_goal, speed, via=via)
        why = report if qs is None else self.clear(qs)
        if why is None:
            print(f"    RMRC {label}: {report}")
            self.run(qs)
            return True
        print(f"    RMRC {label} not usable ({why}) - planning in joint space")
        return self.move_to_tcp(T_goal)

    def move_to_tcp(self, T_tcp):
        q = self.solve(T_tcp)
        if q is None:
            self.faults += 1
            print("    UR3e IK failed")
            return False
        return self.move_joint(q)

    def move_joint(self, q_goal):
        qs, why = ucd.safe_jtraj(self.robot, self.world, q_goal, carried=self.load)
        if qs is None:
            print(f"    collision predicted ({why}) - re-planning")
            rng = np.random.default_rng(1)
            lim = np.asarray(self.robot.qlim)
            for attempt in range(TASK["replan_tries"]):
                q_mid = (self.home if attempt == 0
                         else np.clip(self.home + rng.normal(0, 0.6, self.robot.n), lim[0], lim[1]))
                leg1, w1 = ucd.safe_jtraj(self.robot, self.world, q_mid, carried=self.load)
                if leg1 is None:
                    continue
                leg2, w2 = ucd.safe_jtraj(self.robot, self.world, q_goal, q_start=q_mid,
                                          carried=self.load)
                if leg2 is not None:
                    self.replans += 1
                    print("    re-planned around it")
                    qs = np.vstack((leg1, leg2))
                    break
            else:
                self.faults += 1
                print("    no safe path - holding")
                return False
        self.run(qs)
        return True

    def grip_pose(self, spot, lifted=False):
        depth = TASK["grip_below_rim"]
        z = box_top(spot["floor"]) - depth + (spot["lift"] if lifted else 0.0)
        return SE3(spot["xy"][0], spot["xy"][1], z) * TOOL_DOWN

    def fit_lifts(self):
        """The pad sits near the edge of the UR3e's reach, and how high it can get over
        it depends on the RG2's length, so take the highest approach that solves."""
        out = {}
        for name, spot in spots().items():
            lift = spot["lift"]
            while lift > TASK["min_lift"] and \
                    self.solve(self.grip_pose(dict(spot, lift=lift), lifted=True), q0=self.home) is None:
                lift = round(lift - 0.005, 3)
            out[name] = max(lift, TASK["min_lift"])
            if out[name] < spot["lift"]:
                print(f"   (UR3e approach over the {name} lowered to {out[name] * 1000:.0f} mm - reach limit)")
        return out

    def spots(self):
        return {name: dict(spot, lift=self.lifts[name]) for name, spot in spots().items()}

    # -- picking up and letting go -------------------------------------------
    def attach(self, names):
        F = self.robot.fkine(self.robot.q)
        self.parts = [(n, F.inv() * self.world.get(n).pose) for n in names]
        for n in names:
            self.world.get(n).active = False
        # one exact box around the whole mailer
        p = cfg.PACK
        base = self.world.get("holder_base").pose
        h = p["outer"][2]
        size = (p["outer"][0], p["outer"][1], h)
        centre = base * SE3(0, 0, -p["wall"] / 2 + h / 2)
        self.load = [(size, F.inv() * centre)]

    def detach(self, floor_z):
        """Let go; drop the parts the last millimetre or two so the box sits on floor_z."""
        dz = floor_z - (self.world.get("holder_base").pose.t[2] - cfg.PACK["wall"] / 2)
        for name, _ in self.parts:
            ob = self.world.get(name)
            ob.pose = SE3(0, 0, dz) * ob.pose
            self.shapes[name].T = ob.pose.A
            ob.active = True
        names = self.held()
        self.parts, self.load = [], []
        return names

    # -- the task -------------------------------------------------------------
    def deliver(self):
        """Pad -> spur -> main belt. Returns True when the box is on the conveyor."""
        sp = self.spots()
        pad, spur = sp["pad"], sp["spur"]
        self.sync()
        self.move_jaws(TASK["open_gap"])

        print("6. UR3e picks the boxed card off the outfeed pad")
        over_pad = self.grip_pose(pad, lifted=True)
        via = self.arc_via(self.tcp_pose(self.robot.q), over_pad,
                           TASK["transit_z"], TASK["transit_r"])
        if not self.straight(over_pad, TASK["fast"], "round to the pad", via=via):
            self.say("UR3e can't get over the box - holding")
            return False
        box = ["holder_base"] + self.riders("holder_base")
        with self.ignoring(self.near(pad["xy"], box_top(pad["floor"]))):
            if not self.straight(self.grip_pose(pad), TASK["slow"], "down to the box"):
                self.say("UR3e can't reach down to the box - holding")
                return False
            self.move_jaws(self.grip_gap())
            self.attach(box)
            if not self.straight(self.grip_pose(pad, lifted=True), TASK["slow"], "lift"):
                self.say("UR3e can't lift the box - holding")
                return False
        ok = True

        print("7. UR3e carries it to the infeed belt and sets it down")
        over_spur = self.grip_pose(spur, lifted=True)
        via = self.arc_via(self.tcp_pose(self.robot.q), over_spur,
                           TASK["transit_z"], TASK["transit_r"])
        ok &= self.straight(over_spur, TASK["fast"], "round to the belt", via=via)
        with self.ignoring(self.near(spur["xy"], box_top(spur["floor"]))):
            ok &= self.straight(self.grip_pose(spur), TASK["slow"], "down onto the belt")
            self.box = self.detach(spur["floor"])
            with self.world.ignoring(self.box):         # the jaws start out touching it
                self.move_jaws(TASK["open_gap"])
                ok &= self.straight(self.grip_pose(spur, lifted=True), TASK["slow"],
                                    "clear of the box")

        print("8. UR3e back home, belts carry the box out")
        self.move_joint(self.home)
        self.run_belts()
        self.say("Box is on the conveyor")
        return ok

    def run_belts(self):
        """Spur carries the box out through the hatch onto the main belt, which
        takes it to the outfeed end."""
        s = cfg.SPUR
        here = self.world.get("holder_base").pose.t
        legs = [np.array([0.0, (cfg.CONV_Y - here[1]), 0.0]),
                np.array([TASK["belt_stop_x"] - s["x"], 0.0, 0.0])]
        for leg in legs:
            n = max(2, int(np.linalg.norm(leg) / (TASK["belt_speed"] * TASK["dt"])))
            step = SE3(*(leg / n))
            for _ in range(n):
                for name in self.box:
                    ob = self.world.get(name)
                    ob.pose = step * ob.pose
                    self.shapes[name].T = ob.pose.A
                self.gate.step(TASK["dt"])

    def reset(self):
        self.parts, self.load, self.box = [], [], []
        for ob in self.world.obstacles:
            ob.active = True
        self.robot.q = self.home.copy()
        self.set_jaws(TASK["open_gap"])
        self.sync()
        self.refresh()

    # -- checks ---------------------------------------------------------------
    def reach_report(self):
        print(f"UR3e grip point {np.round(self.tcp.t, 4)} from the flange")
        print(f"{'UR3e pose':18s} {'tcp xyz':26s} IK   collisions")
        sp = self.spots()
        for name in ("pad", "spur"):
            for lifted in (True, False):
                T = self.grip_pose(sp[name], lifted)
                q = self.solve(T, q0=self.home)
                tag = f"{name}/{'approach' if lifted else 'grasp'}"
                if q is None:
                    print(f"{tag:18s} {np.round(T.t, 3)!s:26s} FAIL")
                    continue
                skip = [] if lifted else self.near(sp[name]["xy"], box_top(sp[name]["floor"]))
                with self.ignoring(skip):
                    hit = ucd.first_collision(self.robot, self.world, q)
                print(f"{tag:18s} {np.round(T.t, 3)!s:26s} OK   {hit or 'none'}")
        print(f"home collisions: {ucd.first_collision(self.robot, self.world, self.home) or 'none'}")


def load_robot():
    """The scene's UR3eRG2 if it imports, else a plain UR3e DH model (for checks)."""
    import importlib
    spec = cfg.ROBOTS[ARM]
    try:
        module_name, class_name = spec["loader"].split(":")
        robot = getattr(importlib.import_module(module_name), class_name)()
    except Exception as e:                       # noqa: BLE001 - any failure -> stand-in
        print(f"({spec['loader']} not available: {e}; using a plain UR3e DH model)")
        from roboticstoolbox import DHRobot, RevoluteDH
        d = (0.15185, 0, 0, 0.13105, 0.08535, 0.0921)
        a = (0, -0.24355, -0.2132, 0, 0, 0)
        al = (pi / 2, 0, 0, pi / 2, -pi / 2, 0)
        lim = [-2 * pi, 2 * pi]
        robot = DHRobot([RevoluteDH(d=d[i], a=a[i], alpha=al[i], qlim=lim) for i in range(6)],
                        name="UR3e")
    robot.base = SE3(*spec["base_xyz"]) * SE3.RPY(list(spec["base_rpy"]))
    return robot


if __name__ == "__main__":
    arm = OutfeedArm(None, load_robot(), {}, NoGate())
    arm.reach_report()
