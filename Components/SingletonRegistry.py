from abc import ABCMeta
from typing import Type

class SingletonMeta(ABCMeta):
    """ Ensures correct singleton behavior for all component subclasses. 
        As it stands, we only need one representation of each type of class. 
        Therefore we implemented singleton behaviors for all components so that they can be referenced globally.
        calls to __new__ or __init__ are overridden to reference original object.\n

        After the initial bringup of the components, they are locked to ensure that we do not accidentially try and create/reference a part that does not exist in the system.    
    """
    _locked = False

    def __call__(cls, *args, **kwargs):
        if cls._locked:
            raise RuntimeError(f"Cannot instantiate {cls.__name__} after the lock is triggered.")
        if cls not in _singleton_instances:
            _singleton_instances[cls] = super().__call__(*args, **kwargs)
        return _singleton_instances[cls]
    
    @classmethod
    def lock(cls):
        """Lock the singleton class to prevent further instantiation/initialization."""
        cls._locked = True

    @classmethod
    def unlock(cls):
        """Unlocks the singleton class to allow more instantiations."""
        cls._locked = False
    
# Importing components here is neccesary to prevent circular imports. 
# Component class require SingletonMeta Class 
from .Component import Component
from ..World.SimEnvironment import SimEnvironment 

"""singleton_instances holds all singleton objects."""
_singleton_instances: dict[Type[Component], Component] = {}

_singleton_classes = [Component, SimEnvironment]
    
def get_singleton(cls: Type[Component]) -> Component:
    """ Returns an existing singleton instance. Instances will be created in the ComponentBringup Routine and nowhere else.
        ALL OTHER USER DEFINED routines should get obj references via this method
    """
    if not issubclass(cls, Component) and cls not in _singleton_classes:
        raise TypeError(f"{cls} is not a valid singleton class.") 
    
    if cls not in _singleton_instances:
        raise KeyError(f"{cls} has not been instantiated yet, PLEASE ENSURE that you are trying to reference the correct object type.")
    return _singleton_instances[cls]

def update_singleton_registry(super_cls: Type[Component], cls_obj):
    _singleton_instances[super_cls] = cls_obj