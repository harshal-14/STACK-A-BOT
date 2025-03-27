""" Commonly re-used functions, constants, QoL functions."""

import typing
import numpy as np
import math

from scipy.spatial.transform import Rotation as R

"""Time conversions"""
NS_TO_MS = 1e-6
NS_TO_S = 1e-9

MS_TO_NS = 1e6
MS_TO_S = 1e-3

S_TO_MS = 1e3
S_TO_NS = 1e9

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
    # not a very smart method, but it works
    return R.from_quat(quat, scalar_first=True).inv().as_quat(scalar_first=True)

def multiply_quaternion(q_1: np.ndarray, q_2: np.ndarray) -> np.ndarray:
    """ Computes the Hamilton product of two quaternions q_1 and q_2.
        Note that q_1*q_2 IS NOT equivalent to q_2 * q_1 
        Args:
            q_1 (np.ndarray): First quaternion in form [W X Y Z]
            q_2 (np.ndarray): Second quaternion in form [W X Y Z]
        Returns:
            prod (np.ndarray): Hamilton Product of q_1 and q_2
    """
    q_r_0 = q_1[0]*q_2[0] - q_1[1]*q_2[1] - q_1[2]*q_2[2] - q_1[3]*q_2[3]
    q_r_1 = q_1[0]*q_2[1] + q_1[1]*q_2[0] + q_1[2]*q_2[3] - q_1[3]*q_2[2]
    q_r_2 = q_1[0]*q_2[2] - q_1[1]*q_2[3] + q_1[2]*q_2[0] + q_1[3]*q_2[1]
    q_r_3 = q_1[0]*q_2[3] + q_1[1]*q_2[2] - q_1[2]*q_2[1] + q_1[3]*q_2[0]

    return np.array([float(q_r_0),float(q_r_1),float(q_r_2),float(q_r_3)])

def polar_decomp_quaternion(quat) -> tuple[float, np.ndarray]:
    """ Computes the polar representation of a quaternion [W X Y Z] -> [phi, n] using the following equation:
        \n \t q = |q| * e^(n * phi) = |q| * (cos(phi) + n * sin(phi))
        \n See [stack exchange](https://math.stackexchange.com/questions/1496308/how-can-i-express-a-quaternion-in-polar-form#:~:text=Sorted%20by:,sin%CE%B8=v%7Cv%7C) for impl details. 
        Args:
            quat (np.ndarray): quaternion to decompose
        Returns:
            phi (float): polar anglular component
            n (np.ndarray): unit vector of imaginary part of quaternion (3,)
    """
    phi = math.acos(quat[0] / np.linalg.norm(quat))
    # phi2 = math.asin(np.linalg.norm(quat[1:]) / np.linalg.norm(quat)) # two valid methods for computing phi
    n = quat[1:] / np.linalg.norm(quat[1:])
    return phi, n

def exponentiate_quaternion(quat: np.ndarray, exponent: float) -> np.ndarray:
    """ Computes the exponentiation of a quaternion (quat) by a non-negative power (exponent)
        \n the exponentation of a quaternion is done in polar form using the following equation:
        \n \t q^x = |q|^x * e^(n * phi * x) = |q|^x * (cos(phi * x) + n * sin(phi * x))
        \n see [Quaternion Wiki](https://en.wikipedia.org/wiki/Quaternion#Exponential,_logarithm,_and_power_functions) for impl details.
    
        Args:
            quat (np.ndarray): quaternion (base).
            exponent (float): exponent. Must be a real non-negative number
        Returns:
            result_quat (np.ndarray): result of quat^x
    """
    phi, n = polar_decomp_quaternion(quat)
    a = np.linalg.norm(quat) ** exponent * math.cos(exponent*phi)
    v = np.linalg.norm(quat) ** exponent * n * math.sin(exponent*phi)
    return np.concatenate((np.array([a]), v))