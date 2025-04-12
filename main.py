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
from .Runtime_Handling.Routines.Manipulation import ComponentBringup, ComponentShutdown, LinearInterpolationJS, LinearInterpolationTS
from .Runtime_Handling.Routines.Perception import EnvironmentSetup
from .World.Geometry import Pose

from scipy.spatial.transform import Rotation as R

def main(args:dict):
    """Setup components and environment"""
    initial_routines = []
    if args.mode == 'SIM':
        initial_routines.append(EnvironmentSetup.EnvironmentSetup(args.URDF_path))
    initial_routines.append(ComponentBringup.ComponentBringup(args))

    """Add all routines that should run during operation"""
    home_q = np.array([[0], [0], [-np.pi/2], [0], [-np.pi/4], [0]])
    ee_p1  = Pose(R.from_euler('xyz', [0, np.pi, 0]).as_matrix(), [0.3,  0.0, 0.2]) 
    ee_p2 = Pose(R.from_euler('xyz', [0, np.pi, 0]).as_matrix(), [0.0,  0.3, 0.2]) 
    ee_p3 = Pose(R.from_euler('xyz', [0, np.pi, 0]).as_matrix(), [-0.3, 0.0, 0.2])
    # movements in JS
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(np.array([[-np.pi/2], [0], [0], [0], [0], [0]]), 2.5))
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(np.array([[0], [-np.pi/2], [0], [0], [0], [0]]), 2.5))
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(np.array([[0], [0], [-np.pi/2], [0], [0], [0]]), 2.5))
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(np.array([[0], [0], [0], [-np.pi/2], [0], [0]]), 2.5))
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(np.array([[0], [0], [0], [0], [-np.pi/2], [0]]), 2.5))
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(np.array([[0], [0], [0], [0], [0], [-np.pi/2]]), 2.5))
    initial_routines.append(LinearInterpolationJS.LinearInterpolationJS(home_q, 2.5))
    # movements in TS
    initial_routines.append(LinearInterpolationTS.LinearInterpolationTS(ee_p1, 2))
    initial_routines.append(LinearInterpolationTS.LinearInterpolationTS(ee_p2, 2))
    initial_routines.append(LinearInterpolationTS.LinearInterpolationTS(ee_p3, 2))
    initial_routines.append(LinearInterpolationTS.LinearInterpolationTS(ee_p2, 2))
    initial_routines.append(LinearInterpolationTS.LinearInterpolationTS(ee_p1, 2))
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
