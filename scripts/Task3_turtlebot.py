#!/usr/bin/env python3
import os
import mujoco
import mujoco.viewer
import numpy as np
import time

class RobotController:

    def __init__(self, model_path, gravity=True, mode="1" ):

        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.gravity = gravity
        self._initialize_scene()
        
        self.mode=mode
        

    def _initialize_scene(self):
        
        if self.gravity:
            self.model.opt.gravity[:] = [0.0, 0, -9.8]
        else:
            self.model.opt.gravity[:] = [0, 0, 0]
        mujoco.mj_resetData(self.model, self.data)
        

    def run(self):

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            start_time = time.time()
            while viewer.is_running():
                elapsed = time.time() - start_time
                
                
                if self.mode == "1":
                    self.data.ctrl = [100, 100]

                elif self.mode == "2":
                    # Increase speed suddenly at 3 seconds to cause drift
                    if elapsed > 3.0 and elapsed < 4:
                            self.data.ctrl = [300, 0]
                    else:
                        self.data.ctrl = [300,300]

                elif self.mode == "3":
                    self.data.ctrl = [200,200]

                mujoco.mj_step(self.model, self.data)
                viewer.sync()
     

         
if __name__ == "__main__":
    
    # Ask user what they want to see
    mode = input("Enter the simulation mode {1,2,3} (1.slip / 2. drift / 3. collision): ").strip()

    script_dir = os.path.dirname(__file__)
    print("Script directory:", script_dir)
    repo_root = os.path.abspath(os.path.join(script_dir, '../..'))
    print("Repository root:", repo_root)
    src_dir = os.path.join(repo_root, 'src')
    print("src directory:", src_dir)
    robot_folder = os.path.join(src_dir, 'robot_descriptions', 'turtlebot_waffle_pi')
    print("franka_folder:", robot_folder)
    if mode == "1":
        scene_relative_path = os.path.join(robot_folder, 'scene_turtlebot3_waffle_pi_nofric.xml')
        print("Scene relative path:", scene_relative_path)
    elif mode == "2":
        scene_relative_path = os.path.join(robot_folder, 'scene_turtlebot3_waffle_pi.xml')
        print("Scene relative path:", scene_relative_path)
    else:
        scene_relative_path = os.path.join(robot_folder, 'scene_turtlebot3_waffle_pi_wall.xml')
        print("Scene relative path:", scene_relative_path)

    MODEL_PATH = scene_relative_path
    
    simulate_with_gravity = True
    robot_controller = RobotController(MODEL_PATH, gravity=simulate_with_gravity,mode=mode)

    robot_controller.run()
