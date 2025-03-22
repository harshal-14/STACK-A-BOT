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

from .Components.SingletonRegistry import * # very important line
from .Runtime_Handling import RoutineScheduler
from .Runtime_Handling.Routines.Manipulation import ComponentBringup, LinearInterpolationJS, ComponentShutdown
from .Runtime_Handling.Routines.Perception import EnvironmentSetup

def main(args:dict):
    """Setup components and environment"""
    initial_routines = []
    if args.mode == 'SIM':
        initial_routines.append(EnvironmentSetup.EnvironmentSetup(args.URDF_path, 1e-5))
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
    parser.add_argument('--URDF_path', type=str, default='stack_a_bot/World/models/', help="Filepath of the robot's urdf. ")
    parser.add_argument('--meshes_dir', type=str, default='stack_a_bot/World/models/thor_meshes/', help="Directory where the robot's mesh files live. Useful for sim or digital twin.")
    ## TODO add other args we want in this program...

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)
