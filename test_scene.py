"""Headless check - builds the whole scene without opening a browser."""

import numpy as np
import swift

import card_vault_scene as cvs
import scene_config as cfg


def main():
    env = swift.Swift()
    env.launch(realtime=False, headless=True)
    objects, robots = cvs.build_scene(env)

    print(f"static objects : {len(objects)}")
    print(f"cards          : {len(cfg.card_objects())}")
    for key, robot in robots.items():
        state = type(robot).__name__ if robot is not None else "placeholder"
        print(f"{key:10s}     : {state}")
        if robot is not None:
            print(f"             tool at {np.round(robot.fkine(robot.q).t, 3)}")
    env.step(0.0)
    print("scene built OK")


if __name__ == "__main__":
    main()
