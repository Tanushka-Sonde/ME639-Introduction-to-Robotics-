#!/usr/bin/env python3
import os
import mujoco
import mujoco.viewer
import numpy as np

class RobotController:

    def __init__(self, model_path, gravity=True, ic_choice=3):
        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.gravity = gravity
        self.ic_choice = ic_choice
        self._initialize_scene()

    def _initialize_scene(self):
        # Set gravity
        if self.gravity:
            self.model.opt.gravity[:] = [0.0, 0, -9.8]
        else:
            self.model.opt.gravity[:] = [0, 0, 0]

        mujoco.mj_resetData(self.model, self.data)

        # Choose initial condition for pendulum
        # Assuming pendulum's joint is the first in qpos
        if self.ic_choice == 1:     # little to the right
            self.data.qpos[0] = np.deg2rad(10)   # +10 degrees
        elif self.ic_choice == 2:   # little to the left
            self.data.qpos[0] = np.deg2rad(-10)  # -10 degrees
        else:                       # perfectly upright
            self.data.qpos[0] = 0.0

        mujoco.mj_forward(self.model, self.data)  # recompute derived quantities

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(self.model, self.data)
                viewer.sync()


if __name__ == "__main__":
    script_dir = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(script_dir, '../..'))
    src_dir = os.path.join(repo_root, 'src')
    robot_folder = os.path.join(src_dir, 'robot_descriptions', 'pendulum')
    scene_relative_path = os.path.join(robot_folder, 'pendulum.xml')
    MODEL_PATH = scene_relative_path

    simulate_with_gravity = True

    # Prompt user for IC selection
    print("Select initial condition:")
    print("1: Lean a little to the right (+10°)")
    print("2: Lean a little to the left (-10°)")
    print("3: Perfectly upright")
    choice = int(input("Enter choice [1/2/3]: ").strip())

    robot_controller = RobotController(MODEL_PATH, gravity=simulate_with_gravity, ic_choice=choice)
    robot_controller.run()
