import math
import random

import numpy as np
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.bounds import compute_combined_aabb, compute_obb, create_bbox_cache, get_obb_corners
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.core.utils.semantics import remove_all_semantics
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
from .yaml_utils import read_yaml_config
from .services.SpawnWall import wall_spawner
import random

# Add colliders to Gprim and Mesh descendants of the root prim
def add_colliders(root_prim, approx_type="convexHull"):
    # Iterate descendant prims (including root) and add colliders to mesh or primitive types
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            # Physics
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
        # Add mesh specific collision properties only to mesh types
        if desc_prim.IsA(UsdGeom.Mesh):
            # Add mesh collision properties to the mesh (e.g. collider aproximation type)
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set(approx_type)


# Clear any previous semantic data in the stage
def remove_previous_semantics(stage, recursive: bool = False):
    prims = stage.Traverse()
    for prim in prims:
        remove_all_semantics(prim, recursive)


# Register the boxes and materials randomizer graph
def register_scatter_boxes(pallet_prim, assets_root_path, config):
    # Calculate the bounds of the prim to create a scatter plane of its size
    bb_cache = create_bbox_cache()
    bbox3d_gf = bb_cache.ComputeLocalBound(pallet_prim)
    prim_tf_gf = omni.usd.get_world_transform_matrix(pallet_prim)

    # Calculate the bounds of the prim
    bbox3d_gf.Transform(prim_tf_gf)
    range_size = bbox3d_gf.GetRange().GetSize()

    # Get the quaterion of the prim in xyzw format from usd
    prim_quat_gf = prim_tf_gf.ExtractRotation().GetQuaternion()
    prim_quat_xyzw = (prim_quat_gf.GetReal(), *prim_quat_gf.GetImaginary())

    # Create a plane on the pallet to scatter the boxes on
    plane_scale = (range_size[0] * 0.8, range_size[1] * 0.8, 1)
    plane_pos_gf = prim_tf_gf.ExtractTranslation() + Gf.Vec3d(0, 0, range_size[2])
    plane_rot_euler_deg = quat_to_euler_angles(np.array(prim_quat_xyzw), degrees=True)
    scatter_plane = rep.create.plane(
        scale=plane_scale, position=plane_pos_gf, rotation=plane_rot_euler_deg, visible=False
    )

    def scatter_boxes():
        cardboxes = rep.create.from_usd(
            assets_root_path + config["cardbox"]["url"], semantics=[("class", config["cardbox"]["class"])], count=5
        )
        with cardboxes:
            rep.randomizer.scatter_2d(scatter_plane, check_for_collisions=True)
        return cardboxes.node

    rep.randomizer.register(scatter_boxes)


# Register the place cones randomizer graph
def register_cone_placement(forklift_prim, assets_root_path, config):
    # Get the bottom corners of the oriented bounding box (OBB) of the forklift
    bb_cache = create_bbox_cache()
    centroid, axes, half_extent = compute_obb(bb_cache, forklift_prim.GetPrimPath())
    larger_xy_extent = (half_extent[0] * 1.3, half_extent[1] * 1.3, half_extent[2])
    obb_corners = get_obb_corners(centroid, axes, larger_xy_extent)
    bottom_corners = [
        obb_corners[0].tolist(),
        obb_corners[2].tolist(),
        obb_corners[4].tolist(),
        obb_corners[6].tolist(),
    ]

    # Orient the cone using the OBB (Oriented Bounding Box)
    obb_quat = Gf.Matrix3d(axes).ExtractRotation().GetQuaternion()
    obb_quat_xyzw = (obb_quat.GetReal(), *obb_quat.GetImaginary())
    obb_euler = quat_to_euler_angles(np.array(obb_quat_xyzw), degrees=True)

    def place_cones():
        cones = rep.create.from_usd(
            assets_root_path + config["cone"]["url"], semantics=[("class", config["cone"]["class"])]
        )
        with cones:
            rep.modify.pose(position=rep.distribution.sequence(bottom_corners), rotation_z=obb_euler[2])
        return cones.node
    print(
        
    )
    rep.randomizer.register(place_cones)


# Register light randomization graph
def register_lights_placement(forklift_prim, pallet_prim):
    bb_cache = create_bbox_cache()
    combined_range_arr = compute_combined_aabb(bb_cache, [forklift_prim.GetPrimPath(), pallet_prim.GetPrimPath()])
    pos_min = (combined_range_arr[0], combined_range_arr[1], 6)
    pos_max = (combined_range_arr[3], combined_range_arr[4], 7)

    def randomize_lights():
        lights = rep.create.light(
            light_type="Sphere",
            color=rep.distribution.uniform((0.2, 0.1, 0.1), (0.9, 0.8, 0.8)),
            intensity=rep.distribution.uniform(500, 2000),
            position=rep.distribution.uniform(pos_min, pos_max),
            scale=rep.distribution.uniform(5, 10),
            count=3,
        )
        return lights.node

    rep.randomizer.register(randomize_lights)

def register_objects_spawner(objects, assets_root_path,num_frames):
    for object in objects.items():
        object_name = object[0]
        print(object_name)
        object_params = object[1]
        # Generate the individual coordinate arrays
        x_coords = np.random.uniform(-4.5, 4.5, num_frames).astype(float)
        y_coords = np.random.uniform(-4.5, 4.5, num_frames).astype(float)
        z_coords = np.zeros(num_frames).astype(float)

        # Combine them into a list of [x, y, z] elements
        pos = [
        (float(x), float(y), float(z))
        for x, y, z in zip(x_coords, y_coords, z_coords)
        ]
        pos_test = [(-4,0,0),(-4,0,3),(-4,0,2),(-4,0,1)]
        print(rep.distribution.sequence(pos_test))
        def spawn_objects():
            if object_params['type'] == "Isaac":
                objs = rep.create.from_usd(
                    assets_root_path + object_params['url'],semantics=[('class',object_params['class'])],
                    count = object_params['number']
                )
                with objs:
                    rep.modify.pose(
                        position = rep.distribution.choice(pos)
                        )
                print("created " + object_name)
            elif object_params['type'] == "Local":
                objs = rep.create.from_usd(
                    object_params['usd_path'],semantics=[('class',object_params['class'])],
                    count = object_params['number']
                )
                with objs:
                    rep.modify.pose(
                        position = rep.distribution.choice(pos)
                        )
                print("created " + object_name)
    rep.randomizer.register(spawn_objects)


def register_wall_spawner(walls):
    """
    Registers a randomizer function that spawns walls around the forklift_prim.
    Each time `rep.randomizer.spawn_walls()` is triggered, new walls are generated.
    """
    def spawn_walls():        
        # 2) For each wall, pick a random angle and radius, then compute start & end in the XY plane
        for wall in walls.items():
            wall_name = wall[0]
            params = wall[1]
            request = {
            "name": wall_name,
            "world_path": "/World/Walls",  # A parent prim path for all walls
            "start": params["start"],
            "end":   params["end"],
            "height": params["height"],
            }
            
        
        return None  # we return None or the node, but randomizers typically return a node handle

    # Finally, register this spawn_walls function as a randomizer in Replicator
    rep.randomizer.register(spawn_walls)
