from ..Camera import Camera 
import numpy as np
class SimCamera(Camera):

    def __init__(self):
        print("SimCamera")
        pass

    def connect(self, **kwargs) -> int:
        return 0

    def bringup(self, **kwargs) -> int:
        return 0

    def disconnet(self, **kwargs) -> int:
        return 0

    def get_RGB_image(self) -> np.ndarray:
        pass

    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        pass

    def get_depth_data(self):
        pass