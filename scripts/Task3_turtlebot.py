#!/usr/bin/env python3
import os
import mujoco
import mujoco.viewer
import numpy as np
import time

class RobotController:

    def __init__(self, model_path, gravity=True):

        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.gravity = gravity
        self._initialize_scene()
        
        self.WHEEL_RADIUS = 0.033
        self.TRACK_WIDTH = 0.288
        
        # self.model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        # self.model.opt.impratio = 50
        # self.model.opt.solver = mujoco.mjtSolver.mjSOL_PGS
        # self.model.opt.iterations = 50
        # self.model.opt.noslip_iterations = 100

    def _initialize_scene(self):
        
        if self.gravity:
            self.model.opt.gravity[:] = [0.0, 0, -9.8]
        else:
            self.model.opt.gravity[:] = [0, 0, 0]
        mujoco.mj_resetData(self.model, self.data)
        
    def _push_once(model, data, force=(50, 0, 0), torque=(0, 0, 0)):
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_right_link")
        data.xfrc_applied[base_id, :3] = force
        data.xfrc_applied[base_id, 3:] = torque
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_left_link")
        data.xfrc_applied[base_id, :3] = force
        data.xfrc_applied[base_id, 3:] = torque

    def run(self):

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                t0 = time.time()
                
                t = time.time() - t0

                # Push robot at 2.0s
                if 2.0 < t < 5.002:
                    
                    self._push_once(self.model, self.data, force=(0, 100000, 0))
                    
                
                self.data.ctrl = [1000,1000]
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
     

         
if __name__ == "__main__":

    script_dir = os.path.dirname(__file__)
    print("Script directory:", script_dir)
    repo_root = os.path.abspath(os.path.join(script_dir, '../..'))
    print("Repository root:", repo_root)
    src_dir = os.path.join(repo_root, 'src')
    print("src directory:", src_dir)
    robot_folder = os.path.join(src_dir, 'robot_descriptions', 'turtlebot_waffle_pi')
    print("franka_folder:", robot_folder)
    scene_relative_path = os.path.join(robot_folder, 'scene_turtlebot3_waffle_pi.xml')
    print("Scene relative path:", scene_relative_path)

    MODEL_PATH = scene_relative_path
    
    simulate_with_gravity = True
    robot_controller = RobotController(MODEL_PATH, gravity=simulate_with_gravity)

    robot_controller.run()