from isaacsim_msgs.srv import UrdfToUsd
import os

from isaacsim import SimulationApp
import rclpy
from rclpy.node import Node


class ConvertUrdfUsd(Node):
    def __init__(self):
        super().__init__("Convert_URDF")
        self.srv = self.create_service(UrdfToUsd, "convert_urdf", self.process_message)

    def process_message(self, request, response):
        if (request.using_arena_robot == True):
            response.usd_path = f"/Arena4-IsaacSim/robot_models/Arena_rosnav{request.robot_name}.usd"
            for i in range(request.number_robot):
                response.prim_path.append(f"/World/{request.robot_name}_{i}")
        elif (request.using_arena_robot == False):
            if (os.path.isfile(f"/Arena4-IsaacSim/robot_models/Arena_rosnav/User/{request.robot_name}.usd") == False):
                self.simulation_app = SimulationApp({
                    "headless": True})
                import omni.kit.commands
                from omni.importer.urdf import _urdf
                from omni.isaac.core import World
                from isaacsim.core.utils.extensions import get_extension_path_from_name
                # Setting up import configuration:
                status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
                import_config.merge_fixed_joints = False
                import_config.convex_decomp = False
                import_config.import_inertia_tensor = False
                import_config.fix_base = False
                import_config.distance_scale = 1
                import_config.make_default_prim = True
                import_config.default_drive_type = (_urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY)

                # Import URDF, stage_path contains the path the path to the usd prim in the stage.
                omni.kit.commands.execute(
                    "URDFParseAndImportFile",
                    urdf_path=request.urdf_path,
                    dest_path=f"/Arena4-IsaacSim/robot_models/Arena_rosnav/User/{request.robot_name}.usd",
                    import_config=import_config,
                    get_articulation_root=True
                )
                response.usd_path = f"/Arena4-IsaacSim/robot_models/Arena_rosnav/User/{request.robot_name}.usd"
                for i in range(request.number_robot):
                    response.prim_path.append(f"/World/{request.robot_name}_{i}")
            else:
                response.usd_path = f"/Arena4-IsaacSim/robot_models/Arena_rosnav/User/{request.robot_name}.usd"
                for i in range(request.number_robot):
                    response.prim_path.append(f"/World/{request.robot_name}_{i}")

        return response


def main(args=None):
    rclpy.init(args=args)

    convert_urdf_usd = ConvertUrdfUsd()

    rclpy.spin_once(convert_urdf_usd)

    rclpy.shutdown()


if __name__ == "main":
    main()
