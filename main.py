"""Riley Todo List for this branch:
    I want to write a demo program that shows software architecture in action, 
    and contains well written, documented code for people to branch off.
    Behavior includes running a simple movement with a simulated manip and cleanly shutting down.

    Active Items:
        * Flesh out Sim Manip impl for basic operations
        * Write linear interpolated path planning
        * Comment every class with future work and todos...
            * Make sure every implemented func has a docustring, all func headers have types
        * Figure out immediate action items for me and/or team.
        * Implement pytest behavior?
"""

import argparse
import numpy as np

from .Components.SingletonRegistry import * # very important line
from .Runtime_Handling import RoutineScheduler
from .Runtime_Handling.Routines.Manipulation import ComponentBringup, LinearInterpolationJS, ComponentShutdown
from .Runtime_Handling.Routines.Perception import EnvironmentSetup

def main(args:dict):
    """Setup components and environment"""
    initial_routines = []
    if args.mode == 'SIM':
        initial_routines.append(EnvironmentSetup.EnvironmnetSetup(1e-3))
    initial_routines.append(ComponentBringup.ComponentBringup(args))
    RoutineScheduler.RoutineScheduler.run_routines(initial_routines)

    """Add all routines that should run during operation"""
    home_q = np.array([[0], [0], [-np.pi/2], [0], [0], [0]])
    routine_queue = []
    routine_queue.append(LinearInterpolationJS.LinearInterpolationJS(home_q, 3))
    scheduler = RoutineScheduler.RoutineScheduler(routine_queue)
    
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
    ## TODO add other args we want in this program...

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)
