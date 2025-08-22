import os
import re

from ._base import _PathITF


class Scene(_PathITF):

    def sanitize_path_component(self, component: str) -> str:
        if not re.match(r'^[a-zA-Z_]', component):
            return f'_{component}'
        return component

    def path(self, *path: str) -> str:
        return os.path.join(self._base, *map(self.sanitize_path_component, path))

    def pedestrian(self, *name: str) -> str:
        """
        Returns the path to the pedestrian directory in the world.
        """
        return self.path('pedestrians', *name)

    def door(self, *name: str) -> str:
        """
        Returns the path to the door directory in the world.
        """
        return self.path('doors', *name)

    def obstacle(self, *name: str) -> str:
        """
        Returns the path to the obstacle directory in the world.
        """
        return self.path('obstacles', *name)

    def robot(self, *name: str) -> str:
        """
        Returns the path to the robot directory in the world.
        """
        return self.path('robots', *name)

    def floor(self, *name: str) -> str:
        """
        Returns the path to the floor directory in the world.
        """
        return self.path('floors', *name)

    def wall(self, *name: str) -> str:
        """
        Returns the path to the wall directory in the world.
        """
        return self.path('walls', *name)
