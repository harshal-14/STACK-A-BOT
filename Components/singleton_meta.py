from abc import ABCMeta

class SingletonMeta(ABCMeta):
    """ Ensures correct singleton behavior for all component subclasses. 
        As it stands, we only need one representation of each type of class. 
        Therefore we implemented singleton behaviors for all components so that they can be referenced globally.
        calls to __new__ or __init__ are overridden to reference original object.\n

        After the initial bringup of the components, they are locked to ensure that we do not accidentially try and reference a part that does not exist in the system.    
    """
    _locked = False
    _singleton_instances = {}

    def __call__(cls, *args, **kwargs):
        if cls._locked:
            raise RuntimeError(f"Cannot instantiate {cls.__name__} after the lock is triggered.")
        if cls not in cls._singleton_instances:
            cls._singleton_instances[cls] = super().__call__(*args, **kwargs)
        return cls._singleton_instances[cls]
    
    @classmethod
    def lock(cls):
        """Lock the singleton class to prevent further instantiation/initialization."""
        cls._locked = True

    @classmethod
    def unlock(cls):
        """Unlocks the singleton class to allow more instantiations."""
        cls._locked = False