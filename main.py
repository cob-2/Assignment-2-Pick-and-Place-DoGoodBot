import swift
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
from Kawasaki_RS007N.Kawasaki import RS007N

env = swift.Swift()
env.launch(realtime=True)

robot = RS007N(base=SE3(0, 0, 0))
robot.add_to_env(env)

q_goal = [0, pi/4, pi/2, 0, pi/4, 0]
for q in rtb.jtraj(robot.q, q_goal, 60).q:
    robot.q = q
    env.step(0.02)

env.hold()