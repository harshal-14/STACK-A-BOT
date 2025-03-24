from abc import ABC, abstractmethod

import ikpy.chain
import ikpy.inverse_kinematics
from .Component import Component
import numpy as np
from ..World.Geometry import Pose, Point
import ikpy
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
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_file, base_elements=["art1"])
    
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
    def move_ts(self, pose:Pose | Point):
        """ Commands the interface to move end-effector in task-space. 
            Could be as simple as translating to js and calling `move_js()`
            pose must be within the c-space of the robot
            Args:
                pose (Pose | Point): End effector pose for the robot. 
            """
        raise NotImplementedError("move_ts() not implemented by subclass")
    
    @abstractmethod
    def get_joint_values(self) -> np.ndarray:
        """Retrieves current position of actuators in joint-space. Interface indifferent.
            Returns:
                q_array (np.ndarray): Returns a (6,1) np.float32 numpy array of joint angles in RAD.
        """
        raise NotImplementedError("get_joint_values() not implemented by subclass")
    
    """Do we want to implement the following two methods ourselves, or should we find a library to do this for us?"""
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
        qs = joint_array_sanitizer(q_array)
        t_mat = self.chain.forward_kinematics(list(qs))  
        return Pose.from_t(t_mat)

    def IK_Solver(self, ee_pose: Pose | Point) -> np.ndarray:
        """ Solves for the set of joint angles at a given end effector pose.
        The `ee_pose` must be of the following construction:
            * ee_pose.rotation & ee_pose.translation are in reference to the base frame of the Manipuator. 
            * ee_pose.rotation is SO(3) 
            * ee_pose 
            Args:
                ee_pose (Pose | Point): pose of end_effector to solve for. 
            Returns:
                q_array (np.ndarray): Array of six joint values in radians representing the manip position at a given ee_pose.
        """
        type_checker([ee_pose], [[Pose, Point]])
        if type(ee_pose) == Point:
            ee_pose = Pose(None, ee_pose)
            target_position = ee_pose.to_np()
            target_orientation = None
        # init_pose = self.FK_Solver()
        # qs = self.chain.inverse_kinematics(target_position.flatten()) # target_orientation
        qs = ikpy.inverse_kinematics.inverse_kinematic_optimization(self.chain, ee_pose.get_transform(), self.get_joint_values().flatten())
        return np.array(qs)
