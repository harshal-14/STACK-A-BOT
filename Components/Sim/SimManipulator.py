from ..Manipulator import Manipulator 
import numpy as np
import pybullet as p

class SimManipulator(Manipulator):

    def __init__(self, urdf_file:str, meshes_dir:str):
        print("SimManipulator")
        self.boxID = -1
        self.urdf_file = urdf_file
        p.setAdditionalSearchPath(meshes_dir) # ensures that the mesh files are readable
        self.joint_ids = np.linspace(0,5,6, dtype=np.int32)

    def connect(self, **kwargs) -> int:
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxID = p.loadURDF(self.urdf_file,startPos, startOrientation, useFixedBase=1)
        return 0

    def bringup(self, **kwargs) -> int:
        return self.connect(kwargs=kwargs)

    def disconnet(self, **kwargs) -> int:
        return 0

    def go_to(self, q_array: np.ndarray) -> int:
        p.setJointMotorControlArray(self.boxID, self.joint_ids, p.POSITION_CONTROL, q_array)
        return 0 #TODO: find a good retval
    
    def stop(self):
        p.setJointMotorControlArray(self.boxID, self.joint_ids, p.VELOCITY_CONTROL, np.zeros((6,1)))
    
    def get_joint_values(self) -> np.ndarray:
        joint_states = p.getJointStates(self.boxID, self.joint_ids)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        return np.array(joint_positions, dtype=np.float32).reshape(6,1)