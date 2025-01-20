from .robots_models.waffle import Waffle
def assign_robot_model(name,prim_path,model):
    if model == "waffle":
        return Waffle(name,prim_path)