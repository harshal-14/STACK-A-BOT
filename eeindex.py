import pybullet as p
import time
import numpy as np

# PID Controller class with wrap-around logic for continuous joints
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, is_continuous=False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.is_continuous = is_continuous  # Flag for continuous joints

    def update(self, current_value):
        error = self.setpoint - current_value
        if self.is_continuous:
            error = (error + np.pi) % (2 * np.pi) - np.pi  # Wrap error to [-π, π]
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# Helper function to convert angles
def convert_angle(angle):
    """
    Convert an angle from [180°, 360°] to [-180°, 0°].
    """
    if angle > np.pi:  
        return angle - 2 * np.pi  
    return angle

# Function to zoom the camera to fit the robot and box
def zoom_camera_to_fit():
    robot_position, _ = p.getBasePositionAndOrientation(robot_id)
    box_position_current, _ = p.getBasePositionAndOrientation(box_id)

    center_position = [
        (robot_position[0] + box_position_current[0]) / 2,
        (robot_position[1] + box_position_current[1]) / 2,
        (robot_position[2] + box_position_current[2]) / 2
    ]

    distance = np.linalg.norm(np.array(robot_position) - np.array(box_position_current)) * 2.5

    p.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=45,          # Changes the view angle
        cameraPitch=-15,
        cameraTargetPosition=center_position
    )

# Function to track the EE position
def track_end_effector():
    ee_position, _ = p.getLinkState(robot_id, end_effector_link_index)[:2]
    print(f"EE Position: X={ee_position[0]:.3f}, Y={ee_position[1]:.3f}, Z={ee_position[2]:.3f}")
    return ee_position

# Function to disable collisions between the EE and the box
def disable_collisions():
    p.setCollisionFilterPair(robot_id, box_id, end_effector_link_index, -1, enableCollision=0)
    print("Collisions between EE and box disabled.")

# Function to enable collisions between the EE and the box
def enable_collisions():
    p.setCollisionFilterPair(robot_id, box_id, end_effector_link_index, -1, enableCollision=1)
    print("Collisions between EE and box enabled.")

# Function to attach the box to the end effector
def attach_box_to_ee(box_id, box_height):  
    ee_state = p.getLinkState(robot_id, end_effector_link_index)
    ee_pos = ee_state[0]  # Position
    ee_orn = ee_state[1]  # Orientation

    # Get the box's center of mass and orientation
    box_pos, box_orn = p.getBasePositionAndOrientation(box_id)

    # Calculate the offset based on the box height
    offset = box_height / 2  # Offset is half the height of the box

    # Create a fixed constraint between the EE and the box
    constraint_id = p.createConstraint(
        robot_id, end_effector_link_index, box_id, -1, p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],  # Relative to EE frame
        childFramePosition=[0, 0, offset],  # Relative to box frame (center of mass)
        parentFrameOrientation=ee_orn,
        childFrameOrientation=box_orn  # Align with box orientation
    )
    print(f"Box {box_id} attached to EE with offset {offset}.")
    return constraint_id

# Function to detach the box from the end effector
def detach_box_from_ee(constraint_id):
    p.removeConstraint(constraint_id)
    print("Box detached from EE.")
    enable_collisions()

# Function to reset the robot to its start position
def reset_robot_to_start():
    print("\nResetting robot to start position...")
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=start_joint_angles[i], force=500)

    # Wait for the robot to reach the start position
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

# Control loop to refine EE position using PID controller
def control_loop_pid(target_position, slow_motion=False):
    tolerance = 0.01  # Stop adjusting when within this distance

    # Initialize PID controllers for each joint
    joint_pid_controllers = []
    for i in range(num_joints):
        if i == 0:
            # Joint 0 (continuous) with lower Kp and wrap-around
            if slow_motion:
                joint_pid_controllers.append(PIDController(Kp=2.0, Ki=0.0, Kd=1.0, setpoint=0.0, is_continuous=True))  
            else:
                joint_pid_controllers.append(PIDController(Kp=5.0, Ki=0.0, Kd=2.0, setpoint=0.0, is_continuous=True))
        else:
            if slow_motion:
                joint_pid_controllers.append(PIDController(Kp=1.0, Ki=0.0, Kd=0.5, setpoint=0.0)) 
            else:
                joint_pid_controllers.append(PIDController(Kp=5.0, Ki=0.0, Kd=2.0, setpoint=0.0))

    while True:
        p.stepSimulation()
        if slow_motion:
            time.sleep(0.00005)  # Slower timestep (20 Hz)
        else:
            time.sleep(1.0 / 240.0)  # Normal timestep (240 Hz)

        # Calculate desired joint angles using IK
        desired_joint_angles = p.calculateInverseKinematics(
            robot_id,
            end_effector_link_index,
            target_position,
            targetOrientation=target_orientation,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=start_joint_angles,
            maxNumIterations=100,
            residualThreshold=1e-4
        )

        # Convert tuple to list
        desired_joint_angles = list(desired_joint_angles)

        # Convert Joint 0 angle to [-180°, 0°] range
        desired_joint_angles[0] = convert_angle(desired_joint_angles[0])

        # Print desired joint angles
        print("\nDesired Joint Angles:")
        for i, angle in enumerate(desired_joint_angles):
            print(f"  Joint {i}: {np.degrees(angle):.2f}°")

        # Control joints using PID
        for i in range(num_joints):
            current_joint_angle = p.getJointState(robot_id, i)[0]
            pid_output = joint_pid_controllers[i].update(desired_joint_angles[i] - current_joint_angle)
            force = 400 if i == 0 else 500  # Higher force for Joint 0
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=desired_joint_angles[i], force=force)

            print(f"PID Output for Joint {i}: {pid_output}")

        # Track EE position
        ee_pos = track_end_effector()

        # Calculate the distance to the target
        distance = np.linalg.norm(np.array(ee_pos) - np.array(target_position))

        # If the end effector is close enough to the target, stop the loop
        if distance < tolerance:
            print("\nEE reached the target accurately!")
            break

#main simulation
try:
    # Start PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)  # Set gravity

    # Load the robot URDF file
    robot_urdf_path = "/home/indra137/rosws/src/thor_arm_description/urdf/thor_robot.urdf"
    robot_id = p.loadURDF(robot_urdf_path, useFixedBase=True)

    # Add the floor
    plane_id = p.createCollisionShape(p.GEOM_PLANE)
    ground_id = p.createMultiBody(0, plane_id)

    # Load the first box URDF file
    box_urdf_path = "/home/indra137/rosws/src/thor_arm_description/urdf/box.urdf"
    box_position = [0.3, 0, 0.05]
    box_id = p.loadURDF(box_urdf_path, basePosition=box_position)

    # Load the second box URDF file
    box2_urdf_path = "/home/indra137/rosws/src/thor_arm_description/urdf/box2.urdf"
    box2_position = [0.3, -0.1, 0.03]  # Position of the second box (offset along Y-axis)
    box2_id = p.loadURDF(box2_urdf_path, basePosition=box2_position)

    # Define the end effector link index
    end_effector_link_index = 5  # since 'art6' is the EE

    # Save initial robot position and joint angles
    num_joints = p.getNumJoints(robot_id)
    start_position = p.getBasePositionAndOrientation(robot_id)[0]
    start_joint_angles = [p.getJointState(robot_id, i)[0] for i in range(num_joints)]

    # Define joint limits in radians
    joint_limits = {
        "art1_yaw": {"min": -2 * np.pi, "max": 2 * np.pi},  # Continuous joint
        "art2_pitch": {"min": np.radians(-89.99), "max": np.radians(89.99)},
        "art3_pitch": {"min": np.radians(-90.00), "max": np.radians(90.00)},
        "art4_roll": {"min": -2 * np.pi, "max": 2 * np.pi},  # Continuous joint
        "art5_pitch": {"min": np.radians(-85.94), "max": np.radians(85.94)},
        "art6_roll": {"min": -2 * np.pi, "max": 2 * np.pi},  # Continuous joint
    }

    # Convert joint limits into lists for PyBullet
    lower_limits = [joint_limits[joint]["min"] for joint in joint_limits]
    upper_limits = [joint_limits[joint]["max"] for joint in joint_limits]
    joint_ranges = [upper_limits[i] - lower_limits[i] for i in range(len(lower_limits))]

    # Default target orientation for the EE
    target_orientation = p.getQuaternionFromEuler([0, np.pi, 0])

    # Zoom the camera
    zoom_camera_to_fit()

    # Step 1: Move EE above Box 1 (avoiding Box 2's height)
    print("\nMoving EE above Box 1...")
    box_position_current, _ = p.getBasePositionAndOrientation(box_id)
    box2_position_current, _ = p.getBasePositionAndOrientation(box2_id)
    target_position = [box_position_current[0], box_position_current[1], box_position_current[2] + 0.06]  # Above Box 2's height
    control_loop_pid(target_position)


    # Step 2: Attach Box 1 to the EE
    print("\nAttaching Box 1 to EE...")
    time.sleep(1.0)
    box1_height = 0.1
    constraint_id = attach_box_to_ee(box_id, box1_height)  # Pass box_id as an argument
    time.sleep(1.0)  # Wait for the physics to stabilize

    # Step 3: Move Box 1 to the drop location via a waypoint
    print("\nMoving Box 1 to the drop location via waypoint...")

    # Define the waypoint and drop position (same location)
    waypoint_position = [0.3, 0.0, 0.2]  # Intermediate position and drop location for Box 1
    final_target_position = [-0.3, 0, 0.14]  # Final target position

    # Move to the waypoint
    print("\nMoving to waypoint...")
    control_loop_pid(waypoint_position, slow_motion=True)  # Enable slow motion for stability

    # Stabilize at the waypoint
    for _ in range(100):  # Allow time for stabilization
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    # step 4: Move to the final target position
    print("\nMoving to final target position...")
    control_loop_pid(final_target_position, slow_motion=True)  # Enable slow motion for stability

    # Step 5: Lower EE to ground
    print("\nLowering EE to Box 1...")
    target_position = [final_target_position[0], final_target_position[1], 0.11]  # Slightly above Box 1
    control_loop_pid(target_position, slow_motion=True)

    # Step 5.1: Detach Box 1 from the EE
    print("\nDetaching Box 1 from EE...")
    time.sleep(1.0)
    detach_box_from_ee(constraint_id)

    # Step 6: Move EE above Box 2
    print("\nMoving EE above Box 2...")
    box2_position_current, _ = p.getBasePositionAndOrientation(box2_id)
    target_position = [box2_position[0], box2_position[1], box2_position[2] + 0.04]  # Above Box 2
    control_loop_pid(target_position, slow_motion=True)

    # Step 8: Attach Box 2 to the EE
    print("\nAttaching Box 2 to EE...")
    time.sleep(1.0)
    box2_height = 0.06
    constraint_id = attach_box_to_ee(box2_id, box2_height)  # Pass box2_id as an argument
    time.sleep(1.0)  # Wait for the physics to stabilize

    # Step 8: move to location with waypoint
    print("\nMoving to waypoint...")
    waypoint2_position = [0, -0.3, 0.2]
    control_loop_pid(waypoint2_position, slow_motion=True)  # Enable slow motion for stability

    # Stabilize at the waypoint
    for _ in range(100):  # Allow time for stabilization
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    target2_position = [-0.3, 0, 0.2]  # Final target position

    
    # Step 9: Stack Box 2 on top of Box 1
    print("\nStacking Box 2 on top of Box 1...")
    stack_location = [target2_position[0], target2_position[1], target2_position[2] + 0.01]  # Stack on top of Box 1
    control_loop_pid(stack_location, slow_motion=True)

    # Step 9.1: Lower EE to ground
    print("\nLowering EE above Box 1...")
    target_position = [target2_position[0], target2_position[1], 0.18]  # Slightly above Box 1
    control_loop_pid(target_position, slow_motion=True)

    # Enable collisions between Box 1 and Box 2
    p.setCollisionFilterPair(box_id, box2_id, -1, -1, enableCollision=1)
    print("Collisions between Box 1 and Box 2 enabled.")

    # Step 10: Detach Box 2 from the EE
    print("\nDetaching Box 2 from EE...")
    time.sleep(1.0)
    detach_box_from_ee(constraint_id)

    # Step 11: Return to the start position
    reset_robot_to_start()

    print("\nSimulation will wait for 10 seconds...")
    time.sleep(10)  # Wait for 10 seconds

except KeyboardInterrupt:
    print("\nSimulation stopped by user.")

finally:
    p.disconnect()
    print("\nSimulation closed.")