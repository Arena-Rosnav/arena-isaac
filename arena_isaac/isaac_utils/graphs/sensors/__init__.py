import abc
import typing
from collections.abc import Sequence


class SensorBase(abc.ABC):
    @abc.abstractmethod
    def simulate(self, base_prim: str) -> typing.Any:
        ...

    @abc.abstractmethod
    def publish(self, base_topic: str) -> typing.Any:
        ...

    def paths(self) -> Sequence[str]:
        return ()

    def destroy(self) -> None:
        """Release non-prim resources (writers, render products). Prims handled by caller."""
        return None
