#!/usr/bin/env python3
import os
import mujoco
import mujoco.viewer
import numpy as np

class RobotController:

    def __init__(self, model_path, gravity=True):

        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.gravity = gravity
        self._initialize_scene()

    def _initialize_scene(self):
        # Initial robot + object setup
        joint_qpos = [0.011, -0.7, -0.01, -2.3, -0.01, 1.56, 0.8]
        finger_qpos = [0.04, 0.04]
        if self.gravity:
            self.model.opt.gravity[:] = [0.0, 0, -9.8]
        else:
            self.model.opt.gravity[:] = [0, 0, 0
                                         
                                         ]
        mujoco.mj_resetData(self.model, self.data)

        
        # Set the initial configuration
        self.data.qpos[:] = joint_qpos + finger_qpos 
        mujoco.mj_forward(self.model, self.data)

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
         
if __name__ == "__main__":

    script_dir = os.path.dirname(__file__)
    print("Script directory:", script_dir)
    repo_root = os.path.abspath(os.path.join(script_dir, '../..'))
    print("Repository root:", repo_root)
    src_dir = os.path.join(repo_root, 'src')
    print("src directory:", src_dir)
    robot_folder = os.path.join(src_dir, 'robot_descriptions', 'franka_emika_panda')
    print("franka_folder:", robot_folder)
    scene_relative_path = os.path.join(robot_folder, 'mjx_scene.xml')
    print("Scene relative path:", scene_relative_path)

    MODEL_PATH = scene_relative_path
    
    simulate_with_gravity = True
    
    while True:
        choice = input("Choose simulation mode:\n1 - With gravity\n2 - Without gravity\nEnter choice: ").strip()
        if choice in ("1", "2"):
            break
        print("Invalid choice! Please enter 1 or 2.")

    simulate_with_gravity = (choice == "1")  # Boolean variable storing the preference

    print(f"Simulation mode: {'With Gravity' if simulate_with_gravity else 'Without Gravity'}")

    robot_controller = RobotController(MODEL_PATH, gravity=simulate_with_gravity)

    robot_controller.run()