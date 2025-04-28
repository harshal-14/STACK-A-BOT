"""
Project Code for WPI Robotic Engineering Capstone Project '25.
This file contains the business logic for our stacking procedure. 
    Designed to work on either a physical THOR robot system or in a simulated pybullet environment. 
    Routines (discrete actions taken by the system) are scheduled and executed FIFO.
    Additional tasks handled in the background include safety monitoring and telemetry logging (WIP) 
"""

import argparse
import numpy as np
import os

from .Components.SingletonRegistry import *
from .Runtime_Handling import RoutineScheduler
from .Runtime_Handling.Routines.Manipulation import ComponentBringup, ComponentShutdown, LinearInterpolationJS, LinearInterpolationTS
from .Runtime_Handling.Routines.Perception import EnvironmentSetup
from .Runtime_Handling.Routines.Perception.DUSt3RTestRoutine import DUSt3RTestRoutine
from .World.Geometry import Pose
from .Runtime_Handling.Routines.Manipulation.MoveAndCapture import MoveAndCaptureJS

from scipy.spatial.transform import Rotation as R

def main(args:dict):
    """Setup components and environment"""
    initial_routines = []
    if args.mode == 'SIM':
        initial_routines.append(EnvironmentSetup.EnvironmentSetup(args.URDF_path))
    initial_routines.append(ComponentBringup.ComponentBringup(args))

    """Add all routines that should run during operation"""
    home_q = np.array([[0], [0], [-np.pi/2], [0], [-np.pi/4], [0]])
    
    # If DUSt3R is enabled, add the DUSt3R routine
    if args.use_dust3r:
        dust3r_routine = DUSt3RTestRoutine(
            num_angles=args.num_angles,
            model_name=args.model,
            output_dir=args.output_dir,
            image_dir=args.image_dir  # This will use existing images if specified
        )
        initial_routines.append(dust3r_routine)
    else:
        # Original movement test routines for basic functionality
        # First move to home position (without capturing)
        initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(home_q, 2.5))
        
        output_dir = os.path.join(args.output_dir, "robot_view_captures")
        os.makedirs(output_dir, exist_ok=True)
        
        # Define joint configurations for different views
        view_joint_configs = [
            # Front view
            np.array([[0], [0], [-np.pi/2], [0], [-np.pi/4], [0]]),
            # Right view
            np.array([[np.pi/4], [0], [-np.pi/2], [0], [-np.pi/4], [0]]),
            # Back view
            np.array([[np.pi/2], [0], [-np.pi/2], [0], [-np.pi/4], [0]]),
            # Left view
            np.array([[-np.pi/4], [0], [-np.pi/2], [0], [-np.pi/4], [0]]),
        ]
        
        # Use MoveAndCaptureJS for positions where you want images
        for i, joint_config in enumerate(view_joint_configs):
            position_name = ["front_view", "right_view", "back_view", "left_view"][i]
            initial_routines.append(MoveAndCaptureJS(joint_config, 2, output_dir, position_name))
        
        # Return to home position (without capturing)
        initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(home_q, 2.5))
    
    scheduler = RoutineScheduler.RoutineScheduler(initial_routines)
    while(scheduler.has_routines()):
        scheduler.run()
        # add any other runtime logic that program needs. 
        # These could be things like the safety daemon, watchdog?, telemetry capture?
    
    """After we have finished all planned Routines, we should move to a safe position, and disconnect."""
    # Return to home position
    home_robot2 = LinearInterpolationJS.LinearInterpolationJS(home_q, 3)
    scheduler.add_routine(home_robot2)
    
    # Gracefully disconnect from components
    shutdown = ComponentShutdown.ComponentShutdown()
    scheduler.add_routine(shutdown)
    
    while(scheduler.has_routines()):
        scheduler.run()
    exit(0)

def parse_args():
    parser = argparse.ArgumentParser(prog="STACK-A-BOT",
                                     description="""WPI Robotic Engineering Capstone Project '25.
                                        Authors: Bhat, H., Blair, R., 
                                                 Kohli, J., Patel, C., 
                                                 Pena, S., Raval, D., 
                                                 Rhodes, J., Virone, A.""")
    parser.add_argument('--mode', type=str, default='SIM', help='Which interface to use. Options: "SIM" (default), "HW"')
    parser.add_argument('--URDF_path', type=str, default='stack_a_bot/World/models/', help="Filepath of the robot's urdf. ")
    parser.add_argument('--meshes_dir', type=str, default='stack_a_bot/World/models/thor_meshes/', help="Directory where the robot's mesh files live. Useful for sim or digital twin.")
    # Add DUSt3R and VoxelGrid arguments
    parser.add_argument('--use_dust3r', action='store_true', help="Whether to use DUSt3R for perception")
    parser.add_argument('--num_angles', type=int, default=4, help="Number of angles for DUSt3R perception")
    parser.add_argument('--model', type=str, default='naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt', help="Model name for DUSt3R")
    parser.add_argument('--output_dir', type=str, default='./output', help="Directory to save outputs")
    parser.add_argument('--image_dir', type=str, default=None, help="Directory with existing images (default: capture new images)")
    parser.add_argument('--use_voxel_grid', action='store_true', help="Whether to use VoxelGrid for perception")
    parser.add_argument('--voxel_size', type=float, default=0.01, help="Voxel size for VoxelGrid")
    
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)