from .scene import Scene
from .file import FileDisk

scene = Scene('/World')
file = FileDisk('/tmp/isaac_sim/')

__all__ = [
    'scene',
    'file',
]
