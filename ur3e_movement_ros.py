import argparse
import time

import numpy as np
from spatialmath import SE3

import ur3e_outfeed as uo
from ur3e_outfeed import TASK, NoGate, OutfeedArm, load_robot


# ----------------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------------
ROS = dict(
    host="192.168.27.1",
    port=9090,
    joint_states="/ur/joint_states",
    command="/ur/scaled_pos_joint_traj_controller/command",
    grip_open="/onrobot/open",
    grip_close="/onrobot/close",
    # ur_robot_driver status (ROS 1, launched in the /ur namespace)
    safety_mode="/ur/ur_hardware_interface/safety_mode",
    program_running="/ur/ur_hardware_interface/robot_program_running",
    play="/ur/ur_hardware_interface/dashboard/play",
)

SAFETY_NORMAL = (1, 2)       # ur_dashboard_msgs/SafetyMode: NORMAL, REDUCED; anything else = stopped
SAFETY_NAMES = {1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE STOP", 4: "RECOVERY",
                5: "SAFEGUARD STOP", 6: "SYSTEM E-STOP", 7: "ROBOT E-STOP", 8: "VIOLATION",
                9: "FAULT", 10: "VALIDATE JOINT ID", 11: "UNDEFINED", 12: "AUTOMATIC MODE SAFEGUARD STOP",
                13: "SYSTEM THREE POSITION ENABLING STOP"}

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

REAL = dict(
    speed_scale=0.5,        # 1.0 = the simulated timing; 0.5 = every move takes twice as long
    start_tol=0.05,         # rad; a path whose first point is further than this from the arm is refused
    end_tol=0.02,           # rad; the arm must end this close to the last point
    settle=0.5,             # s to wait after a trajectory's last point
    grip_wait=1.5,          # s for the RG2 to finish opening/closing
    open_gap=None,          # real RG2 open width (m) for the collision checker; None keeps TASK's
    tcp=None,               # flange -> pad centre (m); None keeps the model's
    recovery_scale=0.5,     # after an e-stop, the rest of the interrupted move runs this much slower again
    bridge_steps=30,        # jtraj points joining the stopped arm back onto its path
)


class EStop(Exception):
    """The robot stopped (e-stop, protective stop, program stopped) during a move."""


class Abort(Exception):
    """The operator chose not to resume after an e-stop."""


def state(text):
    """The run's state, on one clearly marked line."""
    print(f"\n==== STATE: {text} ====")

# Pad and spur measured on the real cell, in the UR3e base frame (m):
#   xy = centre of the box footprint, floor = height of the surface the box sits on.
# None = use the scene_config.py positions.
REAL_SPOTS = None
# REAL_SPOTS = dict(pad=dict(xy=(0.25, 0.00), floor=0.020),
#                   spur=dict(xy=(0.00, -0.30), floor=-0.050))


# ----------------------------------------------------------------------------
# THE ROBOT, THROUGH ROSBRIDGE
# ----------------------------------------------------------------------------
class UR3eROS:
    """Joint states in, trajectories and gripper calls out."""

    def __init__(self, host, port):
        import roslibpy
        self.roslibpy = roslibpy
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()
        if not self.client.is_connected:
            raise RuntimeError(f"can't reach rosbridge at {host}:{port}")
        self._q = None
        self.safety, self.running = None, None
        self.safety_topic = roslibpy.Topic(self.client, ROS["safety_mode"], "ur_dashboard_msgs/SafetyMode")
        self.safety_topic.subscribe(lambda m: setattr(self, "safety", int(m["mode"])))
        self.running_topic = roslibpy.Topic(self.client, ROS["program_running"], "std_msgs/Bool")
        self.running_topic.subscribe(lambda m: setattr(self, "running", bool(m["data"])))
        self.states = roslibpy.Topic(self.client, ROS["joint_states"], "sensor_msgs/JointState")
        self.states.subscribe(self._on_state)
        self.command = roslibpy.Topic(self.client, ROS["command"], "trajectory_msgs/JointTrajectory")
        self.command.advertise()
        time.sleep(0.5)                 # rosbridge drops a publish made right after advertising
        self.joints()                   # waits for the first joint state

    def _on_state(self, msg):
        pos = dict(zip(msg["name"], msg["position"]))
        if all(n in pos for n in JOINT_NAMES):
            self._q = np.array([pos[n] for n in JOINT_NAMES], dtype=float)

    def joints(self, timeout=5.0):
        t0 = time.time()
        while self._q is None and time.time() - t0 < timeout:
            time.sleep(0.02)
        if self._q is None:
            raise RuntimeError(f"no joint state on {ROS['joint_states']}")
        return self._q.copy()

    # -- robot status -----------------------------------------------------------
    def stopped(self):
        if self.safety is not None and self.safety not in SAFETY_NORMAL:
            return SAFETY_NAMES.get(self.safety, f"safety mode {self.safety}")
        if self.running is False:
            return "robot program not running"
        return None

    def wait(self, seconds):
        """Sleep, but raise EStop the moment the robot stops."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            why = self.stopped()
            if why:
                raise EStop(why)
            time.sleep(0.02)

    def wait_until_reset(self):
        """Block until the pendant shows a normal safety mode again (e-stop released,
        robot powered and brakes off)."""
        shown = None
        while self.safety is not None and self.safety not in SAFETY_NORMAL:
            if self.safety != shown:
                shown = self.safety
                print(f"      robot reports {SAFETY_NAMES.get(shown, shown)} - release the e-stop, "
                      "then power on and release the brakes on the pendant")
            time.sleep(0.2)

    def restart_program(self, timeout=20.0):
        """Ask the driver's dashboard to press Play, else wait for someone to press it on the pendant."""
        if self.running is not False:
            return
        try:
            srv = self.roslibpy.Service(self.client, ROS["play"], "std_srvs/Trigger")
            srv.call(self.roslibpy.ServiceRequest({}), timeout=5)
        except Exception as e:                       # noqa: BLE001 - fall back to the pendant
            print(f"      (dashboard play unavailable: {e})")
        print("      waiting for the External Control program (press Play on the pendant if needed)")
        t0 = time.time()
        while self.running is False:
            if time.time() - t0 > timeout:
                raise RuntimeError("robot program didn't start - press Play on the pendant and resume again")
            time.sleep(0.1)
        time.sleep(0.5)                              # let the controller pick up the current pose

    def follow(self, qs, dt):
        """Send the whole path as one trajectory, starting from where the arm is now,
        and wait until it has been played out. Raises EStop if the robot stops."""
        why = self.stopped()
        if why:
            raise EStop(why)
        q_now = self.joints()
        points = [dict(positions=q_now.tolist(), time_from_start=dict(secs=0, nsecs=0))]
        for k, q in enumerate(qs, start=1):
            t = k * dt
            points.append(dict(positions=[float(x) for x in q],
                               time_from_start=dict(secs=int(t), nsecs=int((t % 1) * 1e9))))
        self.command.publish(self.roslibpy.Message(dict(joint_names=JOINT_NAMES, points=points)))
        try:
            self.wait(len(qs) * dt + REAL["settle"])
        except EStop:
            self.stop()                 # don't let the rest play out when the program comes back
            raise
        err = float(np.max(np.abs(self.joints() - np.asarray(qs[-1]))))
        if err > REAL["end_tol"]:
            raise RuntimeError(f"arm stopped {err:.3f} rad short of the path end "
                               "(protective stop, speed slider or program paused?)")

    def stop(self):
        """An empty trajectory makes the controller stop where it is."""
        self.command.publish(self.roslibpy.Message(dict(joint_names=JOINT_NAMES, points=[])))

    def gripper(self, close):
        name = ROS["grip_close"] if close else ROS["grip_open"]
        srv = self.roslibpy.Service(self.client, name, "std_srvs/Trigger")
        res = srv.call(self.roslibpy.ServiceRequest({}), timeout=10)
        if not res["success"]:
            raise RuntimeError(f"{name} failed: {res['message']}")
        time.sleep(REAL["grip_wait"])

    def close(self):
        try:
            self.command.unadvertise()
            for topic in (self.states, self.safety_topic, self.running_topic):
                topic.unsubscribe()
        finally:
            self.client.terminate()


class DryRun:
    """Stands in for the robot: follows every path instantly and prints it."""

    def __init__(self, q, estop_at=None):
        self.q = np.asarray(q, dtype=float).copy()
        self.moves = 0
        self.seconds = 0.0
        self.estop_at = estop_at        # pretend the e-stop is hit half way through this move

    def joints(self):
        return self.q.copy()

    def follow(self, qs, dt):
        self.moves += 1
        if self.moves == self.estop_at:
            self.estop_at = None
            half = len(qs) // 2
            self.q = np.asarray(qs[half], dtype=float).copy()
            print(f"      [dry] E-STOP pressed {half} points into this move")
            raise EStop("ROBOT E-STOP (simulated)")
        self.seconds += len(qs) * dt
        qd = np.max(np.abs(np.diff(np.vstack((self.q, qs)), axis=0))) / dt
        print(f"      [dry] {len(qs)} points, {len(qs) * dt:.1f} s, peak joint speed {qd:.2f} rad/s")
        self.q = np.asarray(qs[-1], dtype=float).copy()

    def stop(self):
        pass

    def wait_until_reset(self):
        print("      [dry] e-stop released, robot powered on")

    def restart_program(self):
        print("      [dry] External Control program playing")

    def gripper(self, close):
        print(f"      [dry] RG2 {'close' if close else 'open'}")

    def close(self):
        pass


# ----------------------------------------------------------------------------
# THE OUTFEED, ON THE REAL ARM
# ----------------------------------------------------------------------------
class RealOutfeedArm(OutfeedArm):
    """OutfeedArm with run() and the jaws going to the robot. No Swift, no shapes:
    the box poses come from scene_config / REAL_SPOTS rather than from the sim."""

    def __init__(self, robot, ur, confirm=True):
        self.ur, self.confirm = ur, confirm
        super().__init__(None, robot, {}, NoGate())
        self.robot.q = self.ur.joints()        # plan from where the arm really is
        self.jaws_closed = None

    # -- where the paths go ---------------------------------------------------
    def run(self, qs):
        qs = np.asarray(qs, dtype=float)
        if len(qs) == 0:
            return
        q_now = self.ur.joints()
        gap = float(np.max(np.abs(qs[0] - q_now)))
        if gap > REAL["start_tol"]:
            raise RuntimeError(f"path starts {gap:.3f} rad from the arm - refusing to send it")
        if self.confirm:
            T = self.tcp_pose(qs[-1]).t
            ans = input(f"      send {len(qs)} points to TCP {np.round(T, 3)}? [Enter = go, q = quit] ")
            if ans.strip().lower().startswith("q"):
                raise KeyboardInterrupt
        scale = REAL["speed_scale"]
        while True:
            try:
                self.send(q_now, qs, scale)
                break
            except EStop as e:
                qs = self.recover(str(e), qs)
                q_now, scale = self.ur.joints(), REAL["speed_scale"] * REAL["recovery_scale"]
        self.robot.q = self.ur.joints()
        self.refresh()

    def send(self, q_now, qs, scale):
        # jtraj paths have a fixed step count, so a long joint move can ask for more
        # than qd_max; stretch the timing until no joint goes faster than that.
        step = float(np.max(np.abs(np.diff(np.vstack((q_now, qs)), axis=0))))
        dt = max(TASK["dt"], step / TASK["qd_max"]) / scale
        self.ur.follow(qs, dt)

    # -- e-stop recovery --------------------------------------------------------
    def recover(self, why, qs):
        state(f"E-STOPPED ({why}) - arm halted, sequence paused")
        self.ur.stop()
        self.ur.wait_until_reset()
        state("WAITING FOR RESET - e-stop released; robot will not move until you say so")
        while True:
            ans = input("      type 'resume' to carry on or 'abort' to stop the sequence: ").strip().lower()
            if ans in ("resume", "abort"):
                break
        if ans == "abort":
            raise Abort("operator aborted after e-stop")
        state("RECOVERING - restarting the robot program and re-planning from the real pose")
        self.ur.restart_program()
        q_now = self.ur.joints()
        self.robot.q = q_now
        self.refresh()
        if self.parts:
            ans = input("      is the box still in the gripper? [y/n] ").strip().lower()
            if not ans.startswith("y"):
                raise Abort("box no longer in the gripper")
        k = int(np.argmin(np.linalg.norm(qs - q_now, axis=1)))
        rest = qs[k + 1:] if k + 1 < len(qs) else qs[-1:]
        bridge, hit = uo.ucd.safe_jtraj(self.robot, self.world, qs[k], q_start=q_now,
                                        steps=REAL["bridge_steps"], carried=self.load)
        if bridge is None:
            raise Abort(f"can't rejoin the path safely ({hit})")
        k2, hit = uo.ucd.path_collision(self.robot, self.world, rest,
                                        uo.ucd.contacts(self.robot, self.world, q_now, self.load), self.load)
        if hit is not None:
            raise Abort(f"rest of the path now blocked ({hit} at step {k2})")
        print(f"      rejoining the path at point {k + 1} of {len(qs)}, at reduced speed")
        state("RUNNING (recovered)")
        return np.vstack((bridge[1:], rest))

    def safe_stop(self):
        """After an abort: take the arm home slowly, if the robot is able to move, and
        open the gripper only if it isn't holding the box; otherwise leave it where it is."""
        holding = bool(self.parts)
        state("ABORTED - returning home" + (" still holding the box" if holding else ""))
        try:
            self.ur.wait_until_reset()
            self.ur.restart_program()
            self.robot.q = self.ur.joints()
            if not holding:                     # never drop a held box mid-air
                self.ur.gripper(close=False)
            qs, why = uo.ucd.safe_jtraj(self.robot, self.world, self.home, carried=self.load)
            if qs is None:
                print(f"      no safe path home ({why}) - jog it home on the pendant")
                return
            self.send(self.robot.q, qs, REAL["speed_scale"] * REAL["recovery_scale"])
            state("STOPPED at home - safe to restart the run"
                  + (" (take the box out of the gripper first)" if holding else ""))
        except (EStop, RuntimeError) as e:
            print(f"      couldn't get home ({e}) - leave it and jog it on the pendant")

    def move_jaws(self, gap, steps=12):
        close = gap < self.opening - 1e-4
        if self.jaws_closed is not close:
            why = getattr(self.ur, "stopped", lambda: None)()
            if why:
                self.recover(why, np.array([self.ur.joints()]))
            self.ur.gripper(close)
            self.jaws_closed = close
        self.set_jaws(gap)

    # -- simulation bookkeeping with no sim behind it -------------------------
    def sync(self):
        self.sync_others()

    def riders(self, name, window=0.30, tol=0.004):
        return []                               # the box is one part here

    def refresh(self):
        g = getattr(self.robot, "gripper", None)
        if g is not None and hasattr(self.robot, "flange_pose"):
            g.update(self.robot.flange_pose())
        T = self.robot.fkine(self.robot.q)
        for name, grip in self.parts:
            self.world.get(name).pose = T * grip

    def detach(self, floor_z):
        names = self.held()
        for name in names:
            self.world.get(name).active = True
        self.parts, self.load = [], []
        return names

    def run_belts(self):
        pass


def real_spots(robot):
    """REAL_SPOTS (UR3e base frame) -> the scene frame the planner works in."""
    out = {}
    for name, s in REAL_SPOTS.items():
        p = (robot.base * SE3(s["xy"][0], s["xy"][1], s["floor"])).t
        out[name] = dict(xy=(float(p[0]), float(p[1])), floor=float(p[2]),
                         lift=TASK["pick_lift" if name == "pad" else "drop_lift"])
    return out


def apply_config(robot):
    if REAL["open_gap"] is not None:
        TASK["open_gap"] = REAL["open_gap"]
    if REAL["tcp"] is not None:
        TASK["tcp"] = REAL["tcp"]
    if REAL_SPOTS is not None:
        spots = real_spots(robot)
        uo.spots = lambda: spots                # OutfeedArm reads the targets through this


def check(robot, ur):
    q = ur.joints()
    T = SE3(robot.base).inv() * robot.fkine(q)
    print("joints (deg):", np.round(np.degrees(q), 2))
    print("flange in the UR3e base frame (m):", np.round(T.t, 4))
    print("flange rpy (deg):", np.round(T.rpy(unit="deg"), 2))
    print("Compare with the pendant (Move tab, feature 'Base', TCP set to the bare flange).")


def main():
    ap = argparse.ArgumentParser(description="UR3e outfeed on the real robot via ROS")
    ap.add_argument("--live", action="store_true", help="move the real robot")
    ap.add_argument("--check", action="store_true", help="read the robot's joints and TCP only")
    ap.add_argument("--no-confirm", action="store_true", help="don't ask before each move")
    ap.add_argument("--host", default=ROS["host"])
    ap.add_argument("--port", type=int, default=ROS["port"])
    ap.add_argument("--estop-at", type=int, default=None, metavar="N",
                    help="dry run only: pretend the e-stop is pressed during move N")
    args = ap.parse_args()

    robot = load_robot()
    apply_config(robot)
    import scene_config as cfg
    home = np.array(cfg.ROBOTS[uo.ARM]["q0"], dtype=float)

    if args.check or args.live:
        ur = UR3eROS(args.host, args.port)
    else:
        ur = DryRun(home, estop_at=args.estop_at)
        print("Dry run - nothing is sent. Use --live to move the robot.")

    try:
        if args.check:
            check(robot, ur)
            return
        arm = RealOutfeedArm(robot, ur, confirm=args.live and not args.no_confirm)
        arm.reach_report()
        state("RUNNING")
        print("5. UR3e to home")
        if not arm.move_joint(arm.home):
            print("UR3e can't get home safely from here - move it closer by hand first")
            return
        try:
            ok = arm.deliver()
        except Abort as e:
            print(f"      {e}")
            arm.safe_stop()
            return
        state("DONE" if ok else "DONE WITH FAULTS - check the log above")
        if isinstance(ur, DryRun):
            print(f"[dry] {ur.moves} trajectories, {ur.seconds:.1f} s of motion")
    except KeyboardInterrupt:
        print("\nStopping the arm")
        ur.stop()
    except Exception:
        ur.stop()
        raise
    finally:
        ur.close()


if __name__ == "__main__":
    main()
