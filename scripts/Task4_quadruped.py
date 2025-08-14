#!/usr/bin/env python3
import os
import mujoco
import mujoco.viewer
import numpy as np

class RobotController:

    def __init__(self, model_path, choice="1"):

        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        
        self.choice = choice
        if choice == "1":
            self.gravity = True
        else:
            self.gravity = False
        
        self._initialize_scene()

    def _initialize_scene(self):
        
        if self.gravity:
            self.model.opt.gravity[:] = [0.0, 0, -9.8]
        else:
            self.model.opt.gravity[:] = [0, 0, 0]
        mujoco.mj_resetData(self.model, self.data)

    import mujoco

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            a=0.01
            while viewer.is_running():
                if self.gravity:
                    mujoco.mj_step(self.model, self.data)
                viewer.sync()

         
if __name__ == "__main__":

    script_dir = os.path.dirname(__file__)
    print("Script directory:", script_dir)
    repo_root = os.path.abspath(os.path.join(script_dir, '../..'))
    print("Repository root:", repo_root)
    src_dir = os.path.join(repo_root, 'src')
    print("src directory:", src_dir)
    robot_folder = os.path.join(src_dir, 'robot_descriptions', 'unitree_go2')
    print("franka_folder:", robot_folder)
    scene_relative_path = os.path.join(robot_folder, 'scene_mjx.xml')
    print("Scene relative path:", scene_relative_path)

    MODEL_PATH = scene_relative_path
    
        
    while True:
        choice = input("Choose:\n1 - Test model with gravity\n2 - Test model without gravity\nEnter choice: ").strip()
        if choice in ("1", "2"):
            break
        print("Invalid choice! Please enter 1, 2 ")

    robot_controller = RobotController(MODEL_PATH, choice=choice)

    robot_controller.run()