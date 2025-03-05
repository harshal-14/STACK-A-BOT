from abc import ABC, abstractmethod
from Component import Component
import numpy as np



class Camera(ABC, Component):
    """Abstract representation of an OAK-D light camera . 
        High-level API call are defined here without implementation specfic behavior. """

    @abstractmethod
    def get_RGB_image(self) -> np.ndarray:
        """ Retrieves image from the central RGBD camera.
            Returns:
                image (np.ndarray): a DxHxW float32 ndarray with D = 3
        """
        raise NotImplementedError("get_RGB_image() not implemented by subclass")
    
    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        """ Retrieves images from the two mono cameras.
            Returns:
                image (tuple[np.ndarray, np.ndarray]): a pair of (DxHxW,DxHxW) float32 ndarrays with D = 1
        """
        raise NotImplementedError("get_mono_images() not implemented by subclass")
    
    def get_depth_data(self): # TODO: What type is returned?
        raise NotImplementedError("get_depth_data() not implemented by subclass")
    
    # TODO: Query perception team to get more input on functionalities.