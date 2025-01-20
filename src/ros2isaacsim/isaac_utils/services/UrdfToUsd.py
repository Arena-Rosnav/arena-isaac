#TODO: Work across different robots % work with yaml
import omni.kit.commands as commands
import yaml
from isaacsim_msgs.srv import UrdfToUsd
import sys
from pathlib import Path
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(parent_dir))
from yaml_utils import read_yaml_config


def urdf_to_usd(request, response):
    name = request.name
    urdf_path = request.urdf_path
    # config_path = request.config_path
        
    status, import_config = commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = False
    import_config.make_default_prim = True
    import_config.distance_scale = 1
    import_config.fix_base = False
    
    
    usd_path = f"/home/ubuntu/arena4_ws/src/arena/isaac/robot_models/{request.name}.usd"
    
    status, stage_path = commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        dest_path=usd_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    
    response.usd_path = usd_path
    return response
    
# Urdf importer service callback.
def convert_urdf_to_usd(controller):
    service = controller.create_service(srv_type=UrdfToUsd, 
                        srv_name='isaac/urdf_to_usd', 
                        callback=urdf_to_usd)
    return service