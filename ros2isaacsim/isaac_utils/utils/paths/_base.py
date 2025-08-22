from __future__ import annotations

import abc
import os


class _PathITF(abc.ABC):
    """
    Interface for path manipulation.
    """
    _base: str

    def __init__(self, base: str) -> None:
        self._base = base

    def __call__(self, *path: str) -> _PathITF:
        """
        Returns a sub-path.
        """
        return self.__class__(os.path.join(self._base, *path))

    @abc.abstractmethod
    def path(self, *path: str) -> str:
        """
        Returns the full path as a string.
        """
        raise NotImplementedError
