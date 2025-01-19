from yaml_utils import read_yaml_config
import numpy as np 
from ImportUsds import usd_importer
from isaacsim_msgs import ImportUsd

def yaml_importer(request, response):
    # Read configuration from YAML file
    yaml_path = request.yaml_path
    config = read_yaml_config(yaml_path)
    
    # Extract parameters
    name = config['robot']['name']
    usd_path = config['robot']['usd_path']
    prim_path = config['robot']['prim_path']
    control = config['robot']['control']
    position = config['robot']['position']
    orientation = config['robot']['orientation']

    # Prepare the request for ImportUsd service
    yaml_request = ImportUsd.Request()
    yaml_request.name = name
    yaml_request.usd_path = usd_path
    yaml_request.prim_path = prim_path
    yaml_request.control = control
    yaml_request.position = np.array(position,dtype=np.float32)
    yaml_request.orientation = np.array(orientation,dtype=np.float32)

    usd_response = usd_importer(yaml_request, response)
    
    # Pass the response back (optional, depending on how you want to structure your service)
    response.ret = usd_response.ret
    return response