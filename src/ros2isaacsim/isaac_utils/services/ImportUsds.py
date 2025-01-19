from omni.isaac.core.utils.prims import set_prim_attribute_value
from isaacsim_msgs.srv import ImportUsd
import numpy as np
def usd_importer(stage, request, response):
    name = request.name
    usd_path = request.usd_path
    prim_path = request.prim_path + "/" + name
    position = request.position
    orientation = request.orientation
    stage.add_reference_to_stage(usd_path, prim_path)
    set_prim_attribute_value(prim_path, attribute_name="xformOp:translate", value=np.array(position))
    set_prim_attribute_value(prim_path, attribute_name="xformOp:orient", value=np.array(orientation))
    response.ret = True
    
def import_usd(controller):
    service = controller.create_service(srv_type=ImportUsd, 
                        srv_name='isaac/import_usd', 
                        callback=usd_importer)
    return service