from isaacsim_msgs.srv import SetLights

from isaac_utils.utils import light

from .utils import Service, on_exception


def set_lights_callback(request: SetLights.Request, response: SetLights.Response) -> SetLights.Response:
    response.ret = list(map(on_exception(False)(light.set_state), request.states))
    return response


set_lights_service = Service(srv_type=SetLights, srv_name='isaac/SetLights', callback=set_lights_callback)

__all__ = ['set_lights_service']
