
### Testing Plan for the HW Manipulator

""" [Test 1] Unpowered reading of Joint Values
    Connect to, and read out Joint Values from HW Manipulator
        Connection should not fail
        Should Receive all 0's. 
"""

""" [Test 2] Unpowered moveJS()
    Connect to, and write joints to arbitrary valid joint values. 
        Connection should not fail
        Should receive success response "ok" from Arduino
"""

"""[Test 3] Powered move_js()
    Turn off Power
    Move Robot to "Home Position", and Turn power on
    Connect to, and use LinearInterpolationJS to move robot to a set non-zero position
        Manipulator Should physically move from Home to non-zero position
"""

"""[Test 4] Powered move_js() and drift analysis
    Turn off Power
    Move Robot to "Home Position", and Turn power on
    Connect to, and use LinearInterpolationJS to move robot to a set non-zero position
    Call LinearInterpolationJS to move robot back to zero position. 
        Manipulator should physically move from home to non-zero position
        Manipulator should return exactly to the home position unimpeded by friction and with no gear slippage
"""

"""[Test 5] Powered move_ts()
    Turn off Power
    Move Robot to "Home Position", and Turn power on
    Connect to, and use LinearInterpolationJS to move robot away from home
    use LinearInterpolationTS to move robot to non-singulatity position
        Manipulator should exhibit smooth motion in both joint space, and task space control modes.  
        Manipulator should NOT attempt to move to a different configuration ("Elbow down" vs "Elbow up") during Task-space interpolation
"""

# TODO: Need to write special routine to test endstops...
"""[Test 6] endstop tests?
    Turn off Power
    Move Robot to "Home Position", and Turn power on
    Connect to, and use {endstop_test_routine}
    Carefully trigger each endstop
        Endstops should react to being pressed,
        Motion from joints should stop when their respective endstop is triggered. 
"""

# TODO: Need to write Routine that accepts a string as a custom msg to send to the Manipulator. See HWManipulator for list of testable GCode commands
"""[Test 7] Custom GCode msg testing routine
    Connect to Robot
    Run {custom_gcode_routine} and observe output
        Output matched desired behavior...
"""

# IF we decide to write the safety Daemon, this is how we would test it. This could be its own test file if we expand its behavior 
"""[Test 7] Unpowered Safety Daemon Checker 
    Connect to, and send faulty (defined by tester) move command. 
        Safety Daemon should prevent faulty move from being executed. 
        Satefy Daemon should gracefully stop Manipulator.
"""