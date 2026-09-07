from __future__ import annotations

import traceback
from collections.abc import Callable

import carb
import rclpy.node


def on_exception(value: object) -> Callable[[Callable[..., object]], Callable[..., object]]:
    def inner(fun: Callable[..., object]) -> Callable[..., object]:
        def wrapper(*args: object, **kwargs: object) -> object:
            try:
                return fun(*args, **kwargs)
            except Exception as e:
                carb.log_error(f"Error in {fun.__name__}: {e}\n{traceback.format_exc()}")
                return value

        return wrapper

    return inner


class Service:
    def __init__(self, srv_type: type, srv_name: str, callback: Callable[..., object], **kwargs: object) -> None:
        self.kwargs = {'srv_type': srv_type, 'srv_name': srv_name, 'callback': callback, **kwargs}

    def create(self, controller: rclpy.node.Node, **kwargs: object) -> None:
        controller.create_service(**{**self.kwargs, **kwargs})


class Subscription:
    def __init__(self, msg_type: type, topic: str, callback: Callable[..., object], **kwargs: object) -> None:
        self.kwargs = {'msg_type': msg_type, 'topic': topic, 'callback': callback, **kwargs}

    def create(self, controller: rclpy.node.Node, **kwargs: object) -> None:
        controller.create_subscription(**{**self.kwargs, **kwargs})
