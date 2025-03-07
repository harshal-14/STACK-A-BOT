import typing


""" Perhaps some thought should go into how we check inputs to function to ensure safety. 
    Do we want a class specific to sanatizing inputs?
    Do we want to make a "one size fits all" function to do this?
"""
def input_checker(inputs: list[typing.Any], types: list[list[type]], sizes: list[tuple]=None):
    """Checks inputs to a function to ensure type and size correspondence. 
        Function should be called as the first line of ANY function. 
        Args:
            inputs (list[Any]): inputs needing to be checked by user.
            types (list[list[type]]): a list of acceptable types for the corresponidng input to be of.
            sizes (list[tuple]): *Optional Parameter*. Useful for checking sizes of arrays or lists.         
        Raises:
            ValueError: Any mismatch on expected type and/or size 
    """
    # TODO: Implement
    pass