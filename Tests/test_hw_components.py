""" File containing test functions for HW Manipulator"""
from ..Components.EndEffector import EndEffector
from ..Components.Manipulator import Manipulator

from ..Components.Hardware.HwInterface import HwInterface
from ..Components.Hardware.HwEndEffector import HwEndEffector
from ..Components.Hardware.HwManipulator import HwManipulator
# only used to verify higher-order logic of these methods. 
# from ..Components.Sim.SimManipulator import SimManipulator
# from ..Runtime_Handling.Routines.Perception.EnvironmentSetup import EnvironmentSetup

from ..Components.SingletonRegistry import update_singleton_registry

from ..Runtime_Handling.Routines.Manipulation.LinearInterpolationJS import LinearInterpolationJS
from ..Runtime_Handling.Routines.Manipulation.LinearInterpolationTS import LinearInterpolationTS
" If and when we implement HW EE routines, we can put them here"
from ..Runtime_Handling.RoutineScheduler import RoutineScheduler

from ..World.Geometry import Pose

from scipy.spatial.transform import Rotation as R
import argparse
import time
import numpy as np


# Globals used for each test
hw_interface = HwInterface() 
end_effector = HwEndEffector()
manip = HwManipulator()

# manip = SimManipulator(urdf_file="stack_a_bot/World/models/thor_robot.urdf", meshes_dir="stack_a_bot/World/models/thor_meshes/")
update_singleton_registry(Manipulator, manip)
update_singleton_registry(EndEffector, end_effector)
# if type(manip) == SimManipulator: RoutineScheduler.run_routines([EnvironmentSetup("stack_a_bot/World/models/")])

scheduler = RoutineScheduler()

# Defining helper functions

def powered_startup():
    input("Power off robot, move it to the neccesary position, and then turn the robot back on. Press any key to continue.")
    print("Take a step back from the robot!")
    time.sleep(2)

def yes_no(msg: str) -> bool:
    """Presents a Yes no prompt to tester.

    Returns T/F based on response.
    """
    key = input(msg)
    return key.lower() == 'y'

def test_interface_zero():
    """ [Test 0] Test connection and message transmission
            Connect to arduino and send/receive msg 
                * Connection should not fail
                * message sent should return "ok" status msg 
    """
    assert(not hw_interface.connected)
    assert(hw_interface.connect_device() == 0)
    assert(hw_interface.connected)

    retstr = hw_interface.tx_rx("~") # resumes control. Does nothing if not stopped, should return "ok" status msg
    assert(retstr == "ok")

    assert(hw_interface.connect_device() == 0) # if already connected should do nothing 
    assert(hw_interface.connected)

def test_end_effector_zero():
    """ [Test 0] Test functionality of vacuum end effector
        Connect to, turn on and off vacuum
            * bringup should not fail
            * status variable should match current end effector state
            * vacuum should be toggled when `set_mode` method called
    
    """
    assert(end_effector.bringup() == 0)
    assert(not end_effector.status)
    assert(not end_effector.get_status())

    end_effector.set_mode(True)

    status_val = end_effector.get_status()
    if not status_val:
        end_effector.set_mode(False)
        print("End Effector should be reported as on")
        assert(False)

    if not yes_no("Is Vacuum on? [Y/n]"):
        end_effector.set_mode(False)
        assert(False)

    time.sleep(3)

    end_effector.set_mode(False)

    status_val = end_effector.get_status()
    if status_val:
        end_effector.set_mode(False)
        print("End Effector should be reported as off")
        assert(False)

    if not yes_no("Is Vacuum off? [Y/n]"):
        end_effector.set_mode(False)
        assert(False)
    
def test_end_effector_one():
    """ [Test 1] Test functionality of vacuum end effector on actual blocks
        Turn off Power\n
        Move Robot to position such that the suction cup is pressing on a block and Turn power on\n
        Connect to, and activate end effector, attaching to block\n
        deactive end_effector and deattach from block.
            * bringup should not fail
            * End effector should successfully grab block when toggled on
            * End effector should successfully drop block when toggled off
    """
    powered_startup()

    assert(end_effector.bringup() == 0)
    assert(not end_effector.status)
    assert(not end_effector.get_status())

    end_effector.set_mode(True)

    status_val = end_effector.get_status()
    if not status_val:
        end_effector.set_mode(False)
        print("End Effector should be reported as on")
        assert(False)

    time.sleep(2)
    if not yes_no("Try to dislodge the block. Is it attached? [Y/n]"):
        end_effector.set_mode(False)
        assert(False)

    end_effector.set_mode(False)
    time.sleep(3)

    status_val = end_effector.get_status()
    if status_val:
        end_effector.set_mode(False)
        print("End Effector should be reported as off")
        assert(False)

    if not yes_no("Try to dislodge the block. Is it still attached? [Y/n]"):
        end_effector.set_mode(False)
        assert(False)

def test_end_effector_two():
    """ [Test 2] Test functionality of vacuum end effector on actual blocks with movement (Routine?)
        Turn off Power\n
        Move Robot to position such that the suction cup is a few centimeters above block. Turn power on\n
        Connect to, activate end effector, and move towards block in joint space. \n
        Attempt to lift block off ground using joint space movement.
        deactive end_effector and deattach from block.
            * bringup should not fail
            * End effector should successfully grab block when toggled on and moved towards block
            * End effector should not drop block when moving off of the ground.
            * End effector should successfully drop block when toggled off in mid air
    """
    # TODO: write this test. We can either use Task Space (moving from home to known block position),
    # or jank it by using task space movement of a few joints to mimic that type of motion. 
    pass

def test_manip_zero():
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

def test_manip_one():
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

def test_manip_two():
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

def test_manip_three():
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
    assert(yes_no("Is the end effector back into the home position [Y/n]"))

def test_manip_four():
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

def test_manip_five():
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

def test_manip_six():
    # TODO: Need to write Routine that accepts a string as a custom msg to send to the Manipulator. See HWManipulator for list of testable GCode commands
    """[Test 6] Custom GCode msg testing routine\n
        Connect to Robot\n
        Run {custom_gcode_routine} and observe output
            * Output matched desired behavior...
    """
    pass

def test_manip_seven():
    # IF we decide to write the safety Daemon, this is how we would test it. This could be its own test file if we expand its behavior 
    """[Test 7] Unpowered Safety Daemon Checker\n
        Connect to, and send faulty (defined by tester) move command. 
            * Safety Daemon should prevent faulty move from being executed. 
            * Satefy Daemon should gracefully stop Manipulator.
    """
    pass

manip_tests = [test_manip_zero, test_manip_one, test_manip_two, test_manip_three, test_manip_four, test_manip_five, test_manip_six, test_manip_seven]
ee_tests = [test_end_effector_zero, test_end_effector_one, test_end_effector_two]
interface_tests = [test_interface_zero]

def main(args):
    comp_test_list = []
    if args.component.lower() == "manipulator":
        comp_test_list = manip_tests
    elif args.component.lower() == "end_effector":
        comp_test_list = ee_tests
    elif args.component.lower() == "interface":
        comp_test_list = interface_tests
    else:
        print(f"Unrecognized component {args.component}.\nAvailable components: ['manipulator', 'end_effector', 'interface']")

    if args.test_num >= len(comp_test_list):
        print(f"test_num {args.test_num} out of range for component {args.component}.\nAvailable tests [0-{len(comp_test_list)-1}]")
        exit(1)
    
    print(f"Running test {args.test_num} on component")
    comp_test_list[args.test_num]()

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test_num", type=int, default=0, help="Dictates which test to run from list, ")
    parser.add_argument("--component", type=str, default='interface', help="Dictates which componet test to run. ['manipulator', 'end_effector', 'interface']")
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    main(args)