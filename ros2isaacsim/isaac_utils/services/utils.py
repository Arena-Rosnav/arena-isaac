import sys
import rclpy.node


def on_exception(value):
    def inner(fun):
        def wrapper(*args, **kwargs):
            try:
                return fun(*args, **kwargs)
            except Exception as e:
                print(f"Error in {fun.__name__}: {e}", file=sys.stderr)
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
