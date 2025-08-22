import abc
import os

from ._base import _PathITF


class _FileBase(_PathITF, abc.ABC):
    @abc.abstractmethod
    def makedirs(self, *path: str) -> None:
        """
        Create directories recursively.
        """
        raise NotImplementedError

    def path(self, *path: str) -> str:
        """
        Returns the full path as a string.
        """
        return os.path.join(self._base, *path)

    def robot(self, *path: str) -> str:
        """
        Returns the path to the robot directory.
        """
        return self.path('robots', *path)


class FileNucleus(_FileBase):
    ...


class FileDisk(_FileBase):
    def makedirs(self, *path: str) -> None:
        """
        Create directories recursively.
        """
        os.makedirs(self.path(*path), exist_ok=True)
