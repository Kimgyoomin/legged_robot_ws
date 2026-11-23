"""Direct height-only Nav2 stack; no legged_bringup dependency."""
from legged_navigation.launch_support import navigation_launch

def generate_launch_description():
    return navigation_launch('only')
