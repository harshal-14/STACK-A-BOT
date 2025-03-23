from abc import ABC, abstractmethod

import ikpy.chain
from .Component import Component
import numpy as np
from ..World.Geometry import Pose, Point
import ikpy
from ..Utilities import size_checker, type_checker

class Manipulator(Component, ABC):
    """ Abstract representation of a robotic manipulator. 
        High-level API call are defined here without implementation specfic behavior. \n
        Manipulator is a singleton object with one of its concrete classes being defined in its place.
        It can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
    """
    def __init__(self, urdf_file: str):
        """ADD"""
        # NOTE: although this works in most cases. For a simple movement it has produced noticeable incorrect positions from ik of current EE pose.
        # could be an FK or IK issue, unsure which just right now...
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_file, base_elements=["art1"])
        print(self.chain)
    
    # TODO: Manipulation team: Define all behavior here, and implement the methods in both Hw and Sim impls. 
    @abstractmethod
    def move_js(self, q_array:np.ndarray) -> int:
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
        raise NotImplementedError("move_js() not implemented by subclass")
    
    @abstractmethod
    def move_ts(self, pose:Pose | Point) -> int:
        """ Commands the interface to move end-effector in task-space. 
            Could be as simple as translating to js and calling `move_js()`
            pose must be within the c-space of the robot
            Args:
                pose (Pose | Point): End effector pose for the robot. 
            Returns:
                status (int): Returning 0 indicates a success, and any non-zero value indicates a failure. 
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
                * values = radians between [-pi/2, pi/2]
            Args:
                q_array (np.ndarray): Array of six joint values in radians representing the manip position to solve for. 
            Returns:
                ee_pose (Pose): pose of the end effector.
        """
        
        type_checker([q_array], [[np.ndarray, list]])
        if type(q_array) == list:
            qs = np.array(q_array) 
        else:
            qs = q_array
        size_checker([qs], [[(6,1), (1,6), (6,)]])
        qs = qs.flatten()
        t_mat = self.chain.forward_kinematics(list(qs)) # NOTE: Assumes that this function returns with no problems. No sanity checking on inputs other than shape and type 
        print(type(t_mat))
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
            target_position = ee_pose.to_trans()
        else:
            target_position = ee_pose.trans
        qs = self.chain.inverse_kinematics(target_position)
        return np.array(qs)
