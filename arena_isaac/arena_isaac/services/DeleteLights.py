from isaacsim_msgs.srv import DeleteLights

from isaac_utils.utils import light

from .utils import Service, on_exception


def delete_lights_callback(request: DeleteLights.Request, response: DeleteLights.Response) -> DeleteLights.Response:
    response.ret = list(map(on_exception(False)(light.delete), request.names))
    return response


delete_lights_service = Service(srv_type=DeleteLights, srv_name='isaac/DeleteLights', callback=delete_lights_callback)

__all__ = ['delete_lights_service']
