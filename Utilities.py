""" Commonly re-used functions, constants, QoL functions."""

import typing
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

"""Time conversions"""
NS_TO_MS = 1e-6
NS_TO_S = 1e-9

MS_TO_NS = 1e6
MS_TO_S = 1e-3

S_TO_MS = 1e3
S_TO_NS = 1e9

""" Perhaps some thought should go into how we check inputs to function to ensure safety. 
    Do we want a class specific to sanatizing inputs?
    Do we want to make a "one size fits all" function to do this?
"""
def type_checker(inputs: list[typing.Any], types: list[list[type]]):
    """Checks inputs to a function to ensure type and size correspondence. 
        Function should be called as the first line of ANY function. 
        Args:
            inputs (list[Any]): inputs needing to be checked by user.
            types (list[list[type]]): a list of acceptable types for the corresponidng input to be of.       
        Raises:
            TypeError: Any mismatch on expected type 
    """
    for i in range(len(inputs)):
        if type(inputs[i]) not in types[i]:
            raise TypeError(f"input {i} of type {type(inputs[i])}, not of acceptable type(s) {types[i]}")
        
def size_checker(inputs: list[np.ndarray], sizes: list[list[type]]):
    """Checks inputs to a function to ensure type and size correspondence. 
        Function should be called as the first line of ANY function. 
        Args:
            inputs (list[Any]): inputs needing to be checked by user.
            sizes (list[list[tuple]]): A list of acceptable sizes for an arrays or lists.       
        Raises:
            ValueError: Any mismatch on expected size 
    """
    for i in range(len(inputs)):
        if inputs[i].shape not in sizes[i]:
            raise TypeError(f"input {i} of size {inputs[i].shape}, not of acceptable size(s) {sizes[i]}")
        
# TODO: We can predefine a list of joint limits, and compare against them THOR already has these defined somewhere...
# We should compare these against instead of blanket [-pi,pi]
def joint_array_sanitizer(input: np.ndarray | list) -> np.ndarray:
    """ Ensures that a given array or list is of the correct construction for input into the manipulator.
        Method first checks type, converts obj into an array, checks size, and then flattens into a (6,)
        Args:
            input (np.ndarray | list): joint-space manipulator position
        Raises:
            TypeError:  See type_checker
            ValueError: see size_checker
    """
    type_checker([input], [[np.ndarray, list]])
    if type(input) == list:
        qs = np.array(input, dtype=np.float32) 
    else:
        qs = input
    size_checker([qs], [[(6,1), (1,6), (6,)]])
    qs = qs.flatten()
    for angle in qs:
        if abs(angle) > np.pi:
            raise ValueError(f"joint-space array contains element not in bounds [-pi, pi] {qs}")
    return qs

def inverse_quaternion(quat: np.ndarray) -> np.ndarray:
    # not a very smart method,
    return R.from_quat(quat, scalar_first=True).inv().as_quat(scalar_first=True)

def multiply_quaternion(q_0: np.ndarray, q_1: np.ndarray) -> np.ndarray:
    q_r_0 = q_0[0]*q_1[0] - q_0[1]*q_1[1] - q_0[2]*q_1[2] - q_0[3]*q_1[3]
    q_r_1 = q_0[0]*q_1[1] + q_0[1]*q_1[0] + q_0[2]*q_1[3] - q_0[3]*q_1[2]
    q_r_2 = q_0[0]*q_1[2] - q_0[1]*q_1[3] + q_0[2]*q_1[0] + q_0[3]*q_1[1]
    q_r_3 = q_0[0]*q_1[3] + q_0[1]*q_1[2] - q_0[2]*q_1[1] + q_0[3]*q_1[0]

    return np.array([float(q_r_0),float(q_r_1),float(q_r_2),float(q_r_3)])

def polar_decomp_quaternion(quat) -> tuple[float, np.ndarray]:
    # a = ||quat|| * cos(phi) -> phi = cos^-1(q / ||quat||)
    # n_hat = v / ||v||
    phi = math.acos(quat[0] / np.linalg.norm(quat))
    phi2 = math.asin(np.linalg.norm(quat[1:]) / np.linalg.norm(quat))
    # phi2 == phi AFAIK
    n_hat = quat[1:] / np.linalg.norm(quat[1:])
    return phi, n_hat

def exponentiate_quaternion(quat: np.ndarray, exponent: float) -> np.ndarray:
    # ||quat||^exponent * (cos(exponent*phi) + n_hat * sin(exponent*phi))
    phi, n_hat = polar_decomp_quaternion(quat)
    a = np.linalg.norm(quat) ** exponent * math.cos(exponent*phi)
    v = np.linalg.norm(quat) ** exponent * n_hat * math.sin(exponent*phi)
    return np.concatenate((np.array([a]), v))