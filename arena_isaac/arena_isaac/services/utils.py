import traceback

import carb
import rclpy.node


def on_exception(value):
    def inner(fun):
        def wrapper(*args, **kwargs):
            try:
                return fun(*args, **kwargs)
            except Exception as e:
                carb.log_error(f"Error in {fun.__name__}: {e}\n{traceback.format_exc()}")
                return value
        return wrapper
    return inner


class Service:
    def __init__(self, srv_type, srv_name, callback, **kwargs):
        self.kwargs = {
            'srv_type': srv_type,
            'srv_name': srv_name,
            'callback': callback,
            **kwargs
        }

    def create(self, controller: rclpy.node.Node, **kwargs):
        controller.create_service(**{**self.kwargs, **kwargs})


class Subscription:
    def __init__(self, msg_type, topic, callback, **kwargs):
        self.kwargs = {
            'msg_type': msg_type,
            'topic': topic,
            'callback': callback,
            **kwargs
        }

    def create(self, controller: rclpy.node.Node, **kwargs):
        controller.create_subscription(**{**self.kwargs, **kwargs})
