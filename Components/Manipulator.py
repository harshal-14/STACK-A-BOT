from abc import ABC, abstractmethod
from .Component import Component
import numpy as np
from ..World.Geometry import Pose

class Manipulator(Component, ABC):
    """ Abstract representation of a robotic manipulator. 
        High-level API call are defined here without implementation specfic behavior. 
    """

    @abstractmethod
    def go_to(self, q_array:np.ndarray) -> int:
        """Sends a command to the interface to move actuators in joint-space. 
            The 'q_array' must be of the following construction:
            * Shape =  (1,6) || (6,1) || (6,)
            * type = np.ndarray || list
            * values = radians between [-pi/2, pi/2]
            Args:
                q_array (np.ndarray): Array of six joint values in radians representing the target position to go to. 
            Returns:
                status (int): Returning 0 indicates a success, and any non-zero value indicates a failure. 
        """
        raise NotImplementedError("GoTo() not implemented by subclass")
    
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
                * values = radians between [-pi/2, pi/2]
            Args:
                q_array (np.ndarray): Array of six joint values in radians representing the manip position to solve for. 
            Returns:
                ee_pose (Pose): pose of the end effector.
        """
        # TODO: Figure out what to return in the case of bad values...
        pass

    def IK_Solver(self, ee_pose: Pose) -> np.ndarray:
        """ Solves for the set of joint angles at a given end effector pose.
        The `ee_pose` must be of the following construction:
            * type = Pose
            * ee_pose.rotation & ee_pose.translation are in reference to the base frame of the Manipuator. 
            * ee_pose.rotation is SO(3) 
            Args:
                ee_pose (Pose): pose of end_effector to solve for. 
            Returns:
                q_array (np.ndarray): Array of six joint values in radians representing the manip position at a given ee_pose.
        """
         # TODO: Figure out what to return in the case of bad values...
        pass
