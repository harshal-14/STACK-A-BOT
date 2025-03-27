"""
Add actual docustring to this...
"""

"""Riley Todo List for this branch:
    Active Items:
        * create demo behavior
            * spawn box somewhere....
            * move lin interpolate to box
            * implement ending routines for clean shutdown
        * Figure out immediate action items as a team
        * Implement pytest behavior?
"""

import argparse
import numpy as np
import sys
import os

from .Components.SingletonRegistry import * # very important line
from .Runtime_Handling import RoutineScheduler
from .Runtime_Handling.Routines.Manipulation import ComponentBringup, LinearInterpolationJS, ComponentShutdown
from .Runtime_Handling.Routines.Perception import EnvironmentSetup

def main(args:dict):
    """Setup components and environment"""
    initial_routines = []
    if args.mode == 'SIM':
        initial_routines.append(EnvironmentSetup.EnvironmentSetup(1e-5))
    initial_routines.append(ComponentBringup.ComponentBringup(args))

    """Add all routines that should run during operation"""
    home_q = np.array([[0], [0], [-np.pi/2], [0], [0], [0]])
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(home_q, 5))

    scheduler = RoutineScheduler.RoutineScheduler(initial_routines)
    while(scheduler.has_routines()):
        scheduler.run()
        # add any other runtime logic that program needs. 
        # These could be things like the safety daemon, watchdog?, telemetry capture?
    while(1):
        pass
    """After we have finished all planned Routines, we should move to a safe position, and disconnect."""
    #TODO, write routine to save any data to disk (images, telemetry, point clouds...) 
    #TODO: write routine to move robot to safe positon for shutdown
    home_robot2 = LinearInterpolationJS.LinearInterpolationJS(home_q, 3)
    # gracefully disconnects from components.
    shutdown = ComponentShutdown.ComponentShutdown()
    
    scheduler.add_routine(home_robot2)
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
    parser.add_argument('--URDF_file', type=str, default='stack_a_bot/thor_arm_description/urdf/thor_robot.urdf', help="Filepath of the robot's urdf. ")
    parser.add_argument('--meshes_dir', type=str, default='stack_a_bot/thor_arm_description/meshes/', help="Directory where the robot's mesh files live. Useful for sim or digital twin.")
    
    # Add DUSt3R and VoxelGrid arguments, to be later on used for the perception system
    parser.add_argument('--use_dust3r', action='store_true', help="Whether to use DUSt3R for perception")
    parser.add_argument('--num_angles', type=int, default=4, help="Number of angles for DUSt3R perception")
    parser.add_argument('--model', type=str, default='dust3r_v1', help="Model name for DUSt3R")
    parser.add_argument('--output_dir', type=str, default='./output', help="Directory to save outputs")
    parser.add_argument('--use_voxel_grid', action='store_true', help="Whether to use VoxelGrid for perception")
    parser.add_argument('--voxel_size', type=float, default=0.01, help="Voxel size for VoxelGrid")

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)
