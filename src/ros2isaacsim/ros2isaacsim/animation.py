from isaacsim import SimulationApp

# Start Isaac Sim
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

import omni
import carb
from pxr import Sdf, Gf
from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils import extensions, prims

extensions.enable_extension("omni.anim.people")
extensions.enable_extension("omni.anim.graph.bundle")
extensions.enable_extension("omni.kit.scripting")
extensions.enable_extension("omni.anim.graph.ui")
extensions.enable_extension("omni.replicator.agent.core")

from omni.isaac.core.prims import XFormPrim
from omni.anim.people.scripts.character_behavior import CharacterBehavior
from omni.kit.scripting import BehaviorScript
from omni.anim.people import PeopleSettings
import omni.anim.graph.core as ag
from omni.anim.people.scripts.global_agent_manager import GlobalAgentManager
# Create the world
world = World()

world.scene.add_default_ground_plane()

# Define character paths
character_name = "F_Business_02"
character_prim_path = f"/World/F_Business_02/female_adult_business_02"
skel_root_prim_path = f"/World/F_Business_02/female_adult_business_02/ManRoot/female_adult_business_02"

# Get the asset path
path = get_assets_root_path()
asset_path = f"{path}/Isaac/People/Characters/{character_name}/{character_name}.usd"
biped_path = f"{path}/Isaac/People/Characters/Biped_Setup.usd"

# Add character to the stage
# add_reference_to_stage(usd_path=asset_path, prim_path=f"/World/Characters/{character_name}")
prims.create_prim(f"/World/{character_name}", "Xform", usd_path = asset_path)

char_xform = XFormPrim(skel_root_prim_path,name = skel_root_prim_path + "_xform")

omni.kit.commands.execute("IsaacSimTeleportPrim", prim_path = skel_root_prim_path , rotation = [0.0, 0.0, 0.0, 0.0])

#Add Python script
omni.kit.commands.execute("ApplyScriptingAPICommand", paths=[Sdf.Path(skel_root_prim_path)])


omni.kit.commands.execute("RefreshScriptingPropertyWindowCommand")

scripts = Sdf.AssetPathArray(["/home/kien/.local/share/ov/pkg/isaac-sim-4.2.0/extscache/omni.anim.people-0.5.0/omni/anim/people/scripts/character_behavior.py"])

character_prim = world.stage.GetPrimAtPath(skel_root_prim_path)

character_prim.GetAttribute("omni:scripting:scripts").Set(scripts)

#Setup Biped setup for animation graph
prim = prims.create_prim("/World" + "/Biped_Setup", "Xform", usd_path=biped_path)
prim.GetAttribute("visibility").Set("invisible")

# Get the animation graph that we are going to add to the person
animation_graph = world.stage.GetPrimAtPath("/World/Biped_Setup/CharacterAnimation/AnimationGraph")

omni.kit.commands.execute("ApplyAnimationGraphAPICommand", 
                          paths=[Sdf.Path(skel_root_prim_path)], 
                          animation_graph_path=Sdf.Path(animation_graph.GetPrimPath()))

commands = [
    "female_adult_business_02 GoTo 5.0 0.0 0.0",
    "female_adult_business_02 Idle 3.0",
    "female_adult_business_02 LookAround"
]

agent_manager = GlobalAgentManager()

agent = CharacterBehavior(prim_path = Sdf.Path(skel_root_prim_path))

print(ag.get_characters())

print(agent.init_character())

agent_manager.add_agent(agent_prim_path = skel_root_prim_path ,agent_object=agent)


agent = agent_manager.get_agent(agent_prim_path = skel_root_prim_path)
print(agent.get_agent_name())


agent_manager.inject_command(agent_prim_path = skel_root_prim_path, command_list = commands)

print(agent.commands)

# Simulation loop
while simulation_app.is_running():
    world.step(render=True)


# Close the simulation
simulation_app.close()

