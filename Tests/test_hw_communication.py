""" File containing test functions for HW Manipulator"""
from ..Components.Manipulator import Manipulator
from ..Components.Hardware.HwManipulator import HwManipulator
from ..Components.Sim.SimManipulator import SimManipulator
from ..Components.SingletonRegistry import update_singleton_registry

from ..Runtime_Handling.Routines.Manipulation.LinearInterpolationJS import LinearInterpolationJS
from ..Runtime_Handling.Routines.Manipulation.LinearInterpolationTS import LinearInterpolationTS
from ..Runtime_Handling.Routines.Perception.EnvironmentSetup import EnvironmentSetup
from ..Runtime_Handling.Routines.Manipulation.ComponentBringup import ComponentBringup
from ..Runtime_Handling.RoutineScheduler import RoutineScheduler

from ..World.Geometry import Pose

from scipy.spatial.transform import Rotation as R
import time
import numpy as np



# Globals used for each test
manip = HwManipulator()
# manip = SimManipulator(urdf_file="stack_a_bot/World/models/thor_robot.urdf", meshes_dir="stack_a_bot/World/models/thor_meshes/")
update_singleton_registry(Manipulator, manip)
if type(manip) == SimManipulator: RoutineScheduler.run_routines([EnvironmentSetup("stack_a_bot/World/models/")])

scheduler = RoutineScheduler()

# Defining helper functions

def powered_startup():
    input("Power off robot, move it to the zero position, and then turn the robot back on. Press any key to continue.")
    print("Take a step back from the robot!")
    time.sleep(2)


def test_zero():
    """ [Test 0] Unpowered reading of Joint Values\n
        Connect to, and read out Joint Values from HW Manipulator
            * Connection should not fail
            * Should Receive all 0's. 
    """
    assert(manip.bringup() == 0)
    time.sleep(1) # give time for the first read of a position to occur
    cur_pos = manip.get_joint_values()
    assert(cur_pos is not None)
    assert(cur_pos.shape == (6,1))
    assert(np.linalg.norm(cur_pos) < 0.01)

    assert(manip.disconnect() == 0)

def test_one():
    """ [Test 1] Unpowered moveJS()\n
        Connect to, and write joints to arbitrary valid joint values.
            * Connection should not fail
            * Should receive success response "ok" from Arduino
    """
    assert(manip.bringup() == 0)
    time.sleep(1) # give time for the first read of a position to occur
    try:
        manip.move_js(np.zeros((6,1)))
    except RuntimeError as e:
        print(e)
        assert(False)

    assert(manip.disconnect() == 0)

def test_two():
    """[Test 2] Powered move_js()\n
        Turn off Power\n
        Move Robot to "Home Position", and Turn power on\n
        Connect to, and use LinearInterpolationJS to move robot to a set non-zero position
            * Manipulator Should physically move from Home to non-zero position
    """
    dst_q = np.array([[np.pi/12], [np.pi/12], [np.pi/12], [np.pi/12], [np.pi/12], [np.pi/12]])
    move_routine = LinearInterpolationJS(dst_q, 3)

    powered_startup()
    assert(manip.bringup() == 0)
    RoutineScheduler.run_routines([move_routine])

    cur_pos = manip.get_joint_values()
    assert(np.linalg.norm(cur_pos-dst_q) < 0.1)
    assert(manip.disconnect() == 0)

def test_three():
    """[Test 3] Powered move_js() and drift analysis\n
        Turn off Power\n
        Move Robot to "Home Position", and Turn power on\n
        Connect to, and use LinearInterpolationJS to move robot to a set non-zero position\n
        Call LinearInterpolationJS to move robot back to zero position. 
            * Manipulator should physically move from home to non-zero position
            * Manipulator should return exactly to the home position unimpeded by friction and with no gear slippage
    """
    dst_q = np.array([[np.pi/12], [np.pi/12], [np.pi/12], [np.pi/12], [np.pi/12], [np.pi/12]])
    zero_q = np.zeros((6,1))
    move_routine = LinearInterpolationJS(dst_q, 3)
    move_back_routine = LinearInterpolationJS(zero_q, 3)

    powered_startup()
    assert(manip.bringup() == 0)
    RoutineScheduler.run_routines([move_routine, move_back_routine])
    
    cur_pos = manip.get_joint_values()

    assert(np.linalg.norm(cur_pos) < 0.01)
    key = input("Is the end effector back into the home position [Y/n]")
    print(key)
    assert(key.lower() == 'y')

def test_four():
    """[Test 4] Powered move_ts()\n
        Turn off Power\n
        Move Robot to "Home Position", and Turn power on\n
        Connect to, and use LinearInterpolationJS to move robot away from home\n
        use LinearInterpolationTS to move robot to non-singulatity position
            * Manipulator should exhibit smooth motion in both joint space, and task space control modes.  
            * Manipulator should NOT attempt to move to a different configuration ("Elbow down" vs "Elbow up") during Task-space interpolation
    """
    js_dst_q = np.array([[-np.pi/12], [-np.pi/12], [-np.pi/12], [-np.pi/12], [-np.pi/12], [-np.pi/12]])
    ee_p1  = Pose(R.from_euler('xyz', [0, np.pi, 0]).as_matrix(), [0.3,  0.0, 0.2])
    move_js = LinearInterpolationJS(js_dst_q, 3)
    move_ts = LinearInterpolationTS(ee_p1, 3)
    
    powered_startup()
    assert(manip.bringup() == 0)
    RoutineScheduler.run_routines([move_js, move_ts])
    p1_q = manip.IK_Solver(ee_p1)
    cur_pos = manip.get_joint_values()
    assert(np.linalg.norm(p1_q-cur_pos) < 0.1)
    
    assert(manip.disconnect() == 0)

def test_five():
    # TODO: Need to write special routine to test endstops...
    """[Test 5] endstop tests?\n
        Turn off Power\n
        Move Robot to "Home Position", and Turn power on\n
        Connect to, and use {endstop_test_routine}\n
        Carefully trigger each endstop
            * Endstops should react to being pressed,
            * Motion from joints should stop when their respective endstop is triggered. 
    """
    pass

def test_six():
    # TODO: Need to write Routine that accepts a string as a custom msg to send to the Manipulator. See HWManipulator for list of testable GCode commands
    """[Test 6] Custom GCode msg testing routine\n
        Connect to Robot\n
        Run {custom_gcode_routine} and observe output
            * Output matched desired behavior...
    """
    pass

def test_seven():
    # IF we decide to write the safety Daemon, this is how we would test it. This could be its own test file if we expand its behavior 
    """[Test 7] Unpowered Safety Daemon Checker\n
        Connect to, and send faulty (defined by tester) move command. 
            * Safety Daemon should prevent faulty move from being executed. 
            * Satefy Daemon should gracefully stop Manipulator.
    """
    pass

def main(args):
    print(f"Running test {args.test_num}")
    if args.test_num == 0:
        test_zero()
    elif args.test_num == 1:
        test_one()
    elif args.test_num == 2:
        test_two()
    elif args.test_num == 3:
        test_three()
    elif args.test_num == 4:
        test_four()
    elif args.test_num == 5:
        test_five()
    elif args.test_num == 6:
        test_six()
    elif args.test_num == 7:
        test_seven()

def parse_args():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--test_num", type=int, default=0, help="Dictates which test to run. Valid are [0-7]")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)