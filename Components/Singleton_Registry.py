from typing import Type
from .Component import Component

_singleton_instances: dict[Type[Component], Component] = {}
    
def get_singleton(cls: Type[Component]) -> Component:
    """Returns an existing singleton instance or creates one if missing."""
    # if not is_singleton_class(cls):
    #     raise TypeError(f"{cls} is not a valid singleton class.")
    
    # print(cls.__bases__)

    if cls not in _singleton_instances:
        _singleton_instances[cls] = cls() # if not already created, create one
    return _singleton_instances[cls]

def update_singleton_registry(super_cls: Type[Component], cls_obj):
    _singleton_instances[super_cls] = cls_obj