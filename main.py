"""Riley Todo List for this branch:
    I want to write a demo program that shows software architecture in action, 
    and contains well written, documented code for people to branch off.
    Behavior includes running a simple movement with a simulated manip and cleanly shutting down.

    Active Items:

        * Figure out Sim Environment stuff. Should we continually step in real-time in a seperate thread?
        * Write linear interpolated path planning
        * Flesh out Sim Manip impl for basic operations
        * Write Routine handler behavior
        * Test behavior in main and ensure no crashes or errors. 
        * Comment every class with future work and todos...
            * Make sure every implemented func has a docustring, all func headers have types
        * Figure out immediate action items for me and/or team.
        * Implement pytest behavior?
"""

import argparse
from Runtime_Handling import RoutineScheduler
from Runtime_Handling.Routines.Manipulation import ComponentBringup, Homing, ComponentShutdown
from Runtime_Handling.Routines.Perception import EnvironmentSetup
import pybullet as p
import time
def main(args:dict):

    initial_routines = []
    if args.mode == 'SIM':
        initial_routines.append(EnvironmentSetup.EnvironmnetSetup())
    initial_routines.append(ComponentBringup.ComponentBringup(args.mode))
    initial_routines.append(Homing.Homing())
    scheduler = RoutineScheduler.RoutineScheduler(initial_routines)

    while(scheduler.has_routines()):
        scheduler.run()
        if args.mode == 'SIM':
            p.stepSimulation()
            time.sleep(1e-3)
        # add any other runtime logic that program needs. 
        # These could be things like the safety daemon, watchdog?, telemetry capture?
    
    """After we have finished all planned Routines, we should move to a safe position, and disconnect."""
    #TODO, write routine to save any data to disk (images, telemetry, point clouds...) 
    #TODO: write routine to move robot to safe positon for shutdown
    home_robot2 = Homing.Homing()
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
    parser.add_argument('--URDF_file', type=str, default='thor_arm_description/urdf/thor_robot.urdf', help="Filepath of the robot's urdf. ")
    parser.add_argument('--meshes_dir', type=str, default='thor_arm_description/meshes/', help="Directory where the robot's mesh files live. Useful for sim or digital twin.")
    ## TODO add other args we want in this program...

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)
