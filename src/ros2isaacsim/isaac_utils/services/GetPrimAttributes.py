from omni.isaac.core.utils.prims import get_prim_at_path
import numpy as np
from isaacsim_msgs.srv import GetPrimAttributes
def get_prim_attributes(request,response):
    name = request.name
    prim = get_prim_at_path(request.prim_path)
    response.translate = np.array(prim.GetAttribute("xformOp:translate").Get(),dtype=np.float32)
    quat = prim.GetAttribute("xformOp:orient").Get()
    response.orient =  np.array([quat.real, quat.imaginary[0], quat.imaginary[1], quat.imaginary[2]], dtype=np.float32)
    response.scale = np.array(prim.GetAttribute("xformOp:scale").Get(),dtype=np.float32)

    return response

def get_prim_attr(controller):
    service = controller.create_service(srv_type=GetPrimAttributes, 
                        srv_name='isaac/get_prim_attributes', 
                        callback=get_prim_attributes)
    return service