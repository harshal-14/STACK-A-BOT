from .Camera import Camera 
import numpy as np
class HWCamera(Camera):

    def __init__(self):
        print("HWCamera")
        pass

    def connect(self, *args):
        pass

    def bringup(self, *args):
        pass

    def get_RGB_image(self):
        pass

    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        pass

    def get_depth_data(self):
        pass