""" Commonly re-used functions, constants, QoL functions."""

import typing
import numpy as np

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
            