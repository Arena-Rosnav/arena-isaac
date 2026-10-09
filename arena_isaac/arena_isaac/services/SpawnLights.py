from isaacsim_msgs.srv import SpawnLights

from isaac_utils.utils import light

from .utils import Service, on_exception


def spawn_lights_callback(request: SpawnLights.Request, response: SpawnLights.Response) -> SpawnLights.Response:
    results = list(map(on_exception((False, False))(light.spawn), request.lights))
    response.ret = [ok for ok, _ in results]
    response.shared = [shared for _, shared in results]
    return response


spawn_lights_service = Service(srv_type=SpawnLights, srv_name='isaac/SpawnLights', callback=spawn_lights_callback)

__all__ = ['spawn_lights_service']
