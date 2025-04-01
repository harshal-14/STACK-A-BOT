from .singleton_meta import SingletonMeta

"""singleton_instances holds all singleton objects."""
_singleton_instances = {}

# Import Component class after defining SingletonMeta to avoid circular imports
from .Component import Component

# Define singleton classes
_singleton_classes = [Component]

# Function to be called after SimEnvironment is defined
def register_sim_environment():
    """Register SimEnvironment class to avoid circular imports."""
    from World.SimEnvironment import SimEnvironment
    global _singleton_classes
    if SimEnvironment not in _singleton_classes:
        _singleton_classes.append(SimEnvironment)
    
def get_singleton(cls):
    """ Returns an existing singleton instance. Instances will be created in the ComponentBringup Routine and nowhere else.
        ALL OTHER USER DEFINED routines should get obj references via this method
    """
    if not issubclass(cls, Component) and cls not in _singleton_classes:
        raise TypeError(f"{cls} is not a valid singleton class.") 
    
    if cls not in _singleton_instances:
        raise KeyError(f"{cls} has not been instantiated yet, PLEASE ENSURE that you are trying to reference the correct object type.")
    return _singleton_instances[cls]

def update_singleton_registry(super_cls, cls_obj):
    _singleton_instances[super_cls] = cls_obj