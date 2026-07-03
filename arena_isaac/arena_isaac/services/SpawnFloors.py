
import omni.usd
from isaac_utils.utils.geom import Scale, Translation
from isaac_utils.utils.material import Material, PhysicsParams
from isaac_utils.utils.mesh import create_cube
from isaac_utils.utils.path import world_path
from isaacsim_msgs.msg import Floor
from isaacsim_msgs.srv import SpawnFloors
from pxr import UsdGeom

from .utils import Service, on_exception

_COLLIDER_THICKNESS = 0.5


@on_exception(False)
def spawn_floor(floor: Floor) -> bool:
    # Get service attributes
    prim_path = world_path(floor.name)
    x_len = floor.x_length
    y_len = floor.y_length
    height = 0.01
    pos = Translation.parse(floor.pos)
    top = pos.z + height
    pos.z += height / 2.0

    scale = Scale(x_len, y_len, height)
    create_cube(
        prim_path=prim_path,
        scale=scale,
        position=pos,
        collide=False,
    )

    if (material := Material.from_msg(floor.material)):
        material.bind_to(prim_path)

    # a thick mesh box is the only floor collider newton both holds and grips on
    collider_path = f'{prim_path}_collider'
    create_cube(
        prim_path=collider_path,
        scale=Scale(x_len, y_len, _COLLIDER_THICKNESS),
        position=Translation(pos.x, pos.y, top - _COLLIDER_THICKNESS / 2.0),
    )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Imageable(stage.GetPrimAtPath(collider_path)).MakeInvisible()

    Material.physics(
        parent_prim_path=world_path(),
        key='ground_default',
        params=PhysicsParams(
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
            combine_mode='min',
        ),
    ).bind_to(collider_path)

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
