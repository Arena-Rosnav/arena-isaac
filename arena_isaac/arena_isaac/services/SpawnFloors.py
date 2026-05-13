import omni

from isaac_utils.utils.geom import Scale, Translation
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.mesh import create_cube
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Floor
from isaacsim_msgs.srv import SpawnFloors

from .utils import Service, on_exception


@on_exception(False)
def spawn_floor(floor: Floor) -> bool:
    # Get service attributes
    prim_path = world_path(floor.name)
    x_len = floor.x_length
    y_len = floor.y_length
    height = 0.01
    pos = Translation.parse(floor.pos)
    pos.z += height / 2.0

    scale = Scale(x_len, y_len, height)
    create_cube(
        prim_path=prim_path,
        scale=scale,
        position=pos,
    )

    if (material := Material.from_msg(floor.material)):
        material.bind_to(prim_path)

    Material.physics(
        parent_prim_path=world_path(),
        key='ground_default',
        params=PhysicsParams(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
            combine_mode='min',
        ),
    ).bind_to(prim_path)

    return True


def spawn_floors_callback(request: SpawnFloors.Request, response: SpawnFloors.Response):
    response.ret = list(map(spawn_floor, request.floors))
    return response


spawn_floors_service = Service(
    srv_type=SpawnFloors,
    srv_name='isaac/SpawnFloors',
    callback=spawn_floors_callback
)

__all__ = ['spawn_floors_service']
