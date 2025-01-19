import omni
import numpy as np
from pxr import Gf, UsdGeom
import math
from isaacsim_msgs.srv import SpawnWall

def wall_spawner(request,response):
    #Get service attributes
    name = request.name
    world_path = request.world_path
    height = request.height
    start = np.append(np.array(request.start),height/2)
    end =  np.append(np.array(request.end),height/2)
    start_vec = Gf.Vec3d(*start)
    end_vec = Gf.Vec3d(*end)
    #fixed attributes
    scale=Gf.Vec3f(*[0.05, 1, height/2])
    color=np.array([.2,.2,0.])
    vector_ab = end - start 

    center = (start_vec + end_vec)/2
    length = np.linalg.norm(start[:2] - end[:2])
    angle = math.atan2(vector_ab[1], vector_ab[0])
    
    #create wall
    stage = omni.usd.get_context().get_stage()
    wall = UsdGeom.Cube.Define(stage, f"{world_path}/{name}")
    wall_transform = UsdGeom.XformCommonAPI(wall)
    wall.AddTranslateOp().Set(center)
    wall_transform.SetScale(scale)
    wall.AddRotateZOp().Set(math.degrees(angle))



    response.ret = True
    return response

def spawn_wall(controller):
    service = controller.create_service(srv_type=SpawnWall, 
                        srv_name='isaac/spawn_wall', 
                        callback=wall_spawner)
    return service