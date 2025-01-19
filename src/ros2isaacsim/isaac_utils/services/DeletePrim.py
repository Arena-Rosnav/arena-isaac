import omni.kit.commands as commands
from isaacsim_msgs.srv import DeletePrim

def prim_deleter(request,response):
    name = request.name
    prim_path = request.prim_path
    commands.execute(
        "IsaacSimDestroyPrim",
        prim_path = prim_path,
    )
    response.ret = True
    return response

def _delete_prim(controller):
    service = controller.create_service(srv_type=DeletePrim, 
                        srv_name='isaac/delete_prim', 
                        callback=prim_deleter)
    return service