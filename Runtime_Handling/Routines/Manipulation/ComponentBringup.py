from ..Routine import Routine
from .. ..Components import Camera, EndEffector, Manipulator, SingletonRegistry, Component
from .. ..Components.Hardware import HwCamera, HwEndEffector, HwManipulator
from .. ..Components.Sim import SimCamera, SimEndEffector, SimManipulator
from .. .Status import Status, Condition
class ComponentBringup(Routine):
    """ Makes all components ready for use.

    Instatiates all singleton components via their constructors, adding them the registry. Depending on the mode, adds either all simulated or physical comps.
    After instantiation, routine calls all comp bringup functions. 
    Registry is then locked so that we cannot accidentially create instances of new components not in the registry.

    Attributes:
        args (dict): User specified runtime arguements (mode, file paths...) 
        comp_list (list): List of components added to registry.  
    """

    def __init__(self, args:dict) -> Status:
        self.args = args
        self.comp_list = []

    def init(self, prev_outputs, parameters = None):
        """Depending on the platform, we either make all simulated or all physical components."""
        Component.Component.unlock()
        if self.args.mode == 'SIM':
            ## initiate all Sim Components and then lock all component types
            cam = SimCamera.SimCamera() 
            ee = SimEndEffector.SimEndEffector()
            manip = SimManipulator.SimManipulator(self.args.URDF_path+"thor_robot.urdf",self.args.meshes_dir)
        elif self.args.mode == 'HW':
            cam = HwCamera.HwCamera() 
            ee = HwEndEffector.HwEndEffector()
            manip = HwManipulator.HwManipulator()
        else:
            return Status(Condition.Fault, 
                          err_msg=f"Wrong mode given, expected ['HW', 'SIM'], given {self.mode}", 
                          err_type=ValueError)
        
        self.comp_list = [cam, ee, manip]
        # Edit Registry to have superclass point to concrete impl obj
        SingletonRegistry.update_singleton_registry(Camera.Camera, cam)
        SingletonRegistry.update_singleton_registry(EndEffector.EndEffector, ee)
        SingletonRegistry.update_singleton_registry(Manipulator.Manipulator, manip)

        # prevent new instantiations from occuring past this point without EXPLICIT knowledge
        Component.Component.lock()
        return Status(Condition.Success)

    def loop(self) -> Status:
        """ This is the only BLOCKING call to loop(). 
            This could be implemented non-blocking by sending calls to bringup() to a seperate thread...
            Only neccesary if we deem it so.
        """
        for comp in self.comp_list:
            if(comp.bringup()):
                return Status(Condition.Fault,
                              err_msg=f"Failure in bringup() in component {type(comp)}",
                              err_type=RuntimeError)
        return Status(Condition.Success)
    
    def end(self) -> tuple[Status, dict]:
        """Nothing to do in this end-step for now..."""
        return Status(Condition.Success), dict()
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        """ If Wrong mode given, fault is unrecoverable. 
            If compononents cannont be connected to, fault is also unrecoverable. 
            """
        raise prev_status.err_type(prev_status.err_msg)