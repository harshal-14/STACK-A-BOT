"""
Project Code for WPI Robotic Engineering Capstone Project '25.
This file contains the business logic for our stacking procedure. 
    Designed to work on either a physical THOR robot system or in a simulated pybullet environment. 
    Routines (discrete actions taken by the system) are scheduled and executed FIFO.
    Additional tasks handled in the background include safety monitoring and telemetry logging (WIP) 
"""

import argparse
import numpy as np

from .Components.SingletonRegistry import *
from .Runtime_Handling import RoutineScheduler
from .Runtime_Handling.Routines.Manipulation import ComponentBringup, GrabBoxRoutine, PlaceBoxRoutine
from .Runtime_Handling.Routines.Manipulation.LinearInterpolationJS import LinearInterpolationJS

def main(args:dict):
    """Setup components and environment"""
    initial_routines = []
    if args.mode == 'SIM':
        from .Runtime_Handling.Routines.Perception import EnvironmentSetup
        initial_routines.append(EnvironmentSetup.EnvironmentSetup(args.URDF_path))
    initial_routines.append(ComponentBringup.ComponentBringup(args))

    """Add all routines that should run during operation"""
    move_time = 5
    # move for photo - pickup box
    initial_routines.append(LinearInterpolationJS(np.array([[0], [np.deg2rad(-30)], [np.deg2rad(80)], [0], [0], [0]]), move_time))
    # initial_routines.append(LinearInterpolationJS(np.array([[0], [0], [0], [0], [0], [0]]), 2.5))
    initial_routines.append(LinearInterpolationJS(np.array([[0], [np.deg2rad(35)], [np.deg2rad(-50)], [0], [0], [0]]), move_time))
    initial_routines.append(GrabBoxRoutine.GrabBoxRoutine(np.array([[0], [np.deg2rad(60)], [np.deg2rad(50)], [0], [0], [0]])))
    initial_routines.append(LinearInterpolationJS(np.array([[0], [0], [0], [np.deg2rad(-180)], [0], [0]]), 5))

    # move for photo - place box
    initial_routines.append(LinearInterpolationJS(np.array([[0], [np.deg2rad(35)], [np.deg2rad(-85)], [np.deg2rad(-180)], [0], [0]]), move_time))
    # initial_routines.append(LinearInterpolationJS(np.array([[0], [0], [0], [np.deg2rad(-180)], [0], [0]]), 5))
    initial_routines.append(LinearInterpolationJS(np.array([[0], [np.deg2rad(-35)], [np.deg2rad(50)], [np.deg2rad(-180)], [0], [0]]), move_time))
    initial_routines.append(PlaceBoxRoutine.PlaceBoxRoutine(drop_pose=np.array([[0], [np.deg2rad(-60)], [np.deg2rad(-60)], [np.deg2rad(-180)], [0], [0]]))) 
    initial_routines.append(LinearInterpolationJS(np.array([[0], [0], [0], [0], [0], [0]]), move_time))    

    scheduler = RoutineScheduler.RoutineScheduler(initial_routines)
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
    # Add DUSt3R and VoxelGrid arguments, to be later on used for the perception system
    parser.add_argument('--use_dust3r', action='store_true', help="Whether to use DUSt3R for perception")
    parser.add_argument('--num_angles', type=int, default=4, help="Number of angles for DUSt3R perception")
    parser.add_argument('--model', type=str, default='dust3r_v1', help="Model name for DUSt3R")
    parser.add_argument('--output_dir', type=str, default='./output', help="Directory to save outputs")
    parser.add_argument('--use_voxel_grid', action='store_true', help="Whether to use VoxelGrid for perception")
    parser.add_argument('--voxel_size', type=float, default=0.01, help="Voxel size for VoxelGrid")
    ## TODO add other args we want in this program...

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)


# Investigate where the end effector ends up at the zero position.
