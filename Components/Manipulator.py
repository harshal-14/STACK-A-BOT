from abc import ABC, abstractmethod

from mr_urdf_loader import loadURDF
from modern_robotics import FKinBody, FKinSpace, IKinBody, IKinSpace, JacobianSpace, JacobianBody

from .Component import Component
import numpy as np
from ..World.Geometry import Pose
from ..Utilities import joint_array_sanitizer, type_checker

class Manipulator(Component, ABC):
    """ Abstract representation of a robotic manipulator. 
        High-level API call are defined here without implementation specfic behavior. \n
        Manipulator is a singleton object with one of its concrete classes being defined in its place.
        It can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
    """
    def __init__(self, urdf_file: str):
        """ Constructor for all Manipulator objects. Concrete Subclasses should call super().__init__() and pass in the required params
            Args:
                urdf_file (str): relative file path of manipulator URDF file 
        """
        # ModernRobotics Library implementation of the kinematic chain. 
        # It is based on Twists and the product of exponentials method for kinematics
        MR=loadURDF(urdf_file)
        self.M  = MR["M"] # The Home Configuration (4,4) [R | T]
        self.Slist  = MR["Slist"] # Space Twists
        self.Blist  = MR["Blist"] # Body Twists
        self.Mlist  = MR["Mlist"] # "mass" information of each Link
        self.Glist  = MR["Glist"] # Spatial Inertia Matrix of links

        self.singularity_eps = 1e-10
    
    # TODO: Manipulation team: Define all behavior here, and implement the methods in both Hw and Sim impls. 
    @abstractmethod
    def move_js(self, q_array:np.ndarray):
        """Sends a command to the interface to move actuators in joint-space. 
            The 'q_array' must be of the following construction:
            * Shape =  (1,6) || (6,1) || (6,)
            * type = np.ndarray || list
            * values = radians between [-pi, pi]
            Args:
                q_array (np.ndarray): Array of six joint values in radians representing the target position to go to. 
            """
        raise NotImplementedError("move_js() not implemented by subclass")
    
    @abstractmethod
    def move_ts(self, pose:Pose):
        """ Commands the interface to move end-effector in task-space. 
            Could be as simple as translating to js and calling `move_js()`
            pose must be within the c-space of the robot
            Args:
                pose (Pose): End effector pose for the robot. 
            """
        raise NotImplementedError("move_ts() not implemented by subclass")
    
    @abstractmethod
    def get_joint_values(self) -> np.ndarray:
        """Retrieves current position of actuators in joint-space. Interface indifferent.
            Returns:
                q_array (np.ndarray): Returns a (6,1) np.float32 numpy array of joint angles in RAD.
        """
        raise NotImplementedError("get_joint_values() not implemented by subclass")
    
    def FK_Solver(self, q_array:np.ndarray) -> Pose:
        """ Solves for the end effector pose given a set of joint angles.
            The 'q_array' must be of the following construction:
                * Shape =  (1,6) || (6,1) || (6,)
                * type = np.ndarray || list
                * values = radians between [-pi, pi]
            Args:
                q_array (np.ndarray): Array of six joint values in radians representing the manip position to solve for. 
            Returns:
                ee_pose (Pose): pose of the end effector.
        """
        # clean up joint array to ensure it fits into correct type for FK calls
        qs = joint_array_sanitizer(q_array)
        # choice of FKinSpace vs FKinBody was arbitrary.
        t_mat = FKinSpace(self.M, self.Slist, qs) 
        return Pose.from_t(t_mat)

    def IK_Solver(self, ee_pose: Pose, init_q_list: np.ndarray | None = None) -> np.ndarray:
        """ Solves for the set of joint angles at a given end effector pose.
            The `ee_pose` must be of the following construction:
            * ee_pose.rotation & ee_pose.translation are in reference to the base frame of the Manipuator. 
            * ee_pose.rotation is SO(3) 
            Args:
                ee_pose (Pose): pose of end_effector to solve for. 
                init_q_list (np.ndarray): list of q values to use for inital guess for solving IK. [(6,1), (6,), (1,6)]
            Returns:
                q_array (np.ndarray): Joint values in radians representing ee_pose in Joint-Space. (6,1)
        """
        type_checker([ee_pose, init_q_list], [[Pose], [np.ndarray, type(None)]])
        if not init_q_list:
            init_q_list = self.get_joint_values().flatten()
        qs_zero = np.zeros((6))
        """The following might be a good place to check for singularity"""
        # is_near = self.near_singularity(init_q_list)

        """ Attempting to find IK solution:
                We try 4 different methods to find a valid solution. 
                We only run subsequent attempts upon a failure to converge.
        """
        qs,    success = IKinBody( self.Blist, self.M, ee_pose.get_transform(), init_q_list, 0.01, 0.001)
        if success:
            return qs.reshape((6,1))
        qs, success = IKinSpace(self.Slist, self.M, ee_pose.get_transform(), init_q_list, 0.01, 0.001)
        if success:
            return qs.reshape((6,1))
        # If attempts using init_q fail, we try the home configuration
        qs, success = IKinBody( self.Blist, self.M, ee_pose.get_transform(),     qs_zero, 0.01, 0.001)
        if success:
            return qs.reshape((6,1))
        qs, success = IKinSpace(self.Slist, self.M, ee_pose.get_transform(),     qs_zero, 0.01, 0.001)
        if success:
            return qs.reshape((6,1))
        # if all attempts fail, raise RuntimeError. 
        # This behavior can change in future if we want our algos to be more robust. 
        #   (return None -> handle Nonetype when calling IK_Solver())
        raise RuntimeError("Inverse Kinematics Failed")
    
    def near_singularity(self, q_list: np.ndarray) -> bool:
        """ Tests a given position for proximity to singularity via determinant of the Jacobian. 
            Args:
                q_list (np.ndarray): Position of EE in joint-space. 
            Returns:
                is_near (bool): Whether the position is within some range epsilon to a singularity 
        """
        jB = np.linalg.det(JacobianBody(self.Blist, q_list))
        jS = np.linalg.det(JacobianSpace(self.Blist, q_list))
        if abs(jB) < self.singularity_eps or abs(jS) < self.singularity_eps:
            return True
        return False 
