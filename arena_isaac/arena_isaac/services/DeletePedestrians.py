from arena_people_msgs.srv import DeletePedestrians
from peds import runtime

from .utils import Service, on_exception


@on_exception(DeletePedestrians.Response.NOT_FOUND)
def remove_person(name: str) -> int:
    found = runtime.delete(name)
    return DeletePedestrians.Response.SUCCESS if found else DeletePedestrians.Response.NOT_FOUND


def delete_pedestrians_callback(request: DeletePedestrians.Request, response: DeletePedestrians.Response) -> DeletePedestrians.Response:
    response.results = [remove_person(name) for name in request.names]
    return response


delete_pedestrians_service = Service(srv_type=DeletePedestrians, srv_name='isaac/DeletePedestrians', callback=delete_pedestrians_callback)

__all__ = ['delete_pedestrians_service']
