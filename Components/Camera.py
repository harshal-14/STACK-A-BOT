from abc import ABC, abstractmethod
from .Component import Component
import numpy as np

class Camera(Component, ABC):
    """Abstract representation of an OAK-D light camera. 
        High-level API call are defined here without implementation specfic behavior.\n
        Camera is a singleton object with one of its concrete classes being defined in its place.
        It can be refered to via `SingletonRegistry.get_singleton(Camera)`. 
    """

    # TODO: Perception Team: Define all Camera behavior here, and implement the methods in both Hw and Sim Camera impls. 
    @abstractmethod
    def get_RGB_image(self) -> np.ndarray:
        """ Retrieves image from the central RGBD camera.
            Returns:
                image (np.ndarray): a DxHxW float32 ndarray with D = 3
        """
        raise NotImplementedError("get_RGB_image() not implemented by subclass")
    
    @abstractmethod
    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        """ Retrieves images from the two mono cameras.
            Returns:
                image (tuple[np.ndarray, np.ndarray]): a pair of (DxHxW,DxHxW) float32 ndarrays with D = 1
        """
        raise NotImplementedError("get_mono_images() not implemented by subclass")
    
    @abstractmethod
    def get_depth_data(self): # TODO: Decide Return type..
        raise NotImplementedError("get_depth_data() not implemented by subclass")
