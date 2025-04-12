from ..Manipulator import Manipulator 
import numpy as np
import pybullet as p
from ...World.Geometry import Pose, Point 
from ...Utilities import joint_array_sanitizer

class SimManipulator(Manipulator):
    """Low-Level Integration of Simulated Manipulator in pybullet

    Pybullet has a lot of manipulator specific integration. Our SimManipulator's methods **must** be generalizable to any Manipulator (e.g IK will be not calculated by pybullets IKSolver).

    Please refer to https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.jxof6bt5vhut for pybullet methods used\n
    SimManipulator is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
        
    Attributes:
        manipulator_ID (int): pybullet object index
        urdf_file (str): file_path holding URDF info for the manipulator
        joint_ids (np.ndarray): Joint indices on robot.
    """
    def __init__(self, urdf_file:str, meshes_dir:str):
        super().__init__(urdf_file)
        print("SimManipulator")
        self.manipulator_ID = -1
        self.urdf_file = urdf_file
        p.setAdditionalSearchPath(meshes_dir) # ensures that the mesh files are readable
        self.joint_ids = np.linspace(0,5,6, dtype=np.int32)

    def __del__(self):
        self.stop()

    def stop(self):
        p.setJointMotorControlArray(self.manipulator_ID, self.joint_ids, p.VELOCITY_CONTROL, np.zeros((6,1)))

    def bringup(self, **kwargs) -> int:
        return self.connect(kwargs=kwargs)

    def connect(self, **kwargs) -> int:
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.manipulator_ID = p.loadURDF(self.urdf_file,startPos, startOrientation, useFixedBase=1)
        return 0

    def disconnect(self, **kwargs) -> int:
        return 0

    def move_js(self, q_array: np.ndarray | list):
        """ Commands a movement in joint-space via pybullet call `setJointMotorControlArray`. 
            Positional control is used by default, which runs a PD controller under the hood.  
            Args:
                q_array (np.ndarray | list): Array of six joint values in radians representing the target position to go to. 
            Returns:
                status (int): Returning 0 indicates a success, and any non-zero value indicates a failure. 
        """
        qs = joint_array_sanitizer(q_array)
        p.setJointMotorControlArray(self.manipulator_ID, self.joint_ids, p.POSITION_CONTROL, qs)
    
    def move_ts(self, pose: Pose):
        """ Commands a movement in task-space by utilzing IK solver and then `move_js`
            Args:
                pose (Pose): Target end-effector position in R^3 w/ rotation
        """            
        ikpy_joints = self.IK_Solver(pose)
        self.move_js(ikpy_joints)
    
    def get_joint_values(self) -> np.ndarray:
        joint_states = p.getJointStates(self.manipulator_ID, self.joint_ids)
        joint_positions = [state[0] for state in joint_states]
        # joint_velocities = [state[1] for state in joint_states]
        return np.array(joint_positions, dtype=np.float32).reshape(6,1)
    
    def _get_ee(self) -> np.ndarray:
        """ THIS Method is for TESTING PURPOSES ONLY.
            Returns:
                link_state (np.ndarray): the end-effector position in cartesian space.
        """
        link_state = p.getLinkState(self.manipulator_ID, 5)
        return np.array(link_state[0])