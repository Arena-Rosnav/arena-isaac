from isaac_utils.yaml_utils import read_yaml_config
import omni.graph.core as og


class Waffle:
    def __init__(self, name, prim_path):
        self.name = name
        self.prim_path = prim_path

    def control_and_publish_joint_states(self):

        og.Controller.edit(
            # default graph name for robots.
            {"graph_path": f"{self.prim_path}/controller"},
            {
                # Create nodes for the OmniGraph
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ROS2SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("ScaleStageUnits", "isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit"),
                    ("Break3Vector_Linear", "omni.graph.nodes.BreakVector3"),
                    ("Break3Vector_Angular", "omni.graph.nodes.BreakVector3"),
                    ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                    ("ConstantToken0", "omni.graph.nodes.ConstantToken"),
                    ("ConstantToken1", "omni.graph.nodes.ConstantToken"),
                    ("MakeArray", "omni.graph.nodes.ConstructArray"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),


                ],

                og.Controller.Keys.SET_VALUES: [
                    # ROS2 domain ID
                    ("ROS2Context.inputs:domain_id", 30),

                    # MakeArray size
                    ("MakeArray.inputs:arraySize", 2),

                    # ROS2 Subscriber for controlling
                    ("ROS2SubscribeTwist.inputs:topicName", f"/{self.name}/cmd_vel"),

                    # DifferentialController
                    ("DifferentialController.inputs:wheelDistance", 0.16),
                    ("DifferentialController.inputs:wheelRadius", 0.033),
                    ("DifferentialController.inputs:maxWheelSpeed", 10.0),
                    ("DifferentialController.inputs:maxLinearSpeed", 2.0),
                    ("DifferentialController.inputs:maxAngularSpeed", 2.0),
                    ("DifferentialController.inputs:maxAcceleration", 0.0),
                    ("DifferentialController.inputs:maxDeceleration", 0.0),
                    ("DifferentialController.inputs:maxAngularAcceleration", 0.0),



                    # PublishJointState
                    ("PublishJointState.inputs:targetPrim", f"{self.prim_path}/base_footprint"),
                    ("PublishJointState.inputs:topicName", f"/{self.name}/joint_states"),

                    # ArticulationController
                    ("ArticulationController.inputs:targetPrim", self.prim_path),
                    ("ConstantToken0.inputs:value", 'wheel_left_joint'),
                    ("ConstantToken1.inputs:value", 'wheel_right_joint'),

                ],

                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("MakeArray.inputs:input1", "token"),
                ],
                # 3) Connect each node's pins
                og.Controller.Keys.CONNECT: [
                    # -- Execution flow
                    ("OnPlaybackTick.outputs:tick", "ROS2SubscribeTwist.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("ROS2SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),


                    # ReadSimTime
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    # -- ROS context to the subscriber
                    ("ROS2Context.outputs:context", "ROS2SubscribeTwist.inputs:context"),
                    ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),

                    # -- Scale the linear velocity before splitting it
                    ("ROS2SubscribeTwist.outputs:linearVelocity", "ScaleStageUnits.inputs:value"),
                    ("ScaleStageUnits.outputs:result", "Break3Vector_Linear.inputs:tuple"),

                    # -- Break the scaled linear velocity into x,y,z
                    ("Break3Vector_Linear.outputs:x", "DifferentialController.inputs:linearVelocity"),

                    # -- Break the angular velocity into x,y,z (only z used typically)
                    ("ROS2SubscribeTwist.outputs:angularVelocity", "Break3Vector_Angular.inputs:tuple"),
                    ("Break3Vector_Angular.outputs:z", "DifferentialController.inputs:angularVelocity"),

                    # -- Constant tokens to MakeArray for joint indices
                    ("ConstantToken0.inputs:value", "MakeArray.inputs:input0"),
                    ("ConstantToken1.inputs:value", "MakeArray.inputs:input1"),
                    ("MakeArray.outputs:array", "ArticulationController.inputs:jointNames"),

                    # -- DifferentialController outputs to ArticulationController
                    ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ],
            }
        )

    def publish_odom_and_tf(self):
        og.Controller.edit(
            {"graph_path": f"{self.prim_path}/Odom_Publisher", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("readSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("publishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("publishRawTF2", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("publishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),

                ],
                og.Controller.Keys.SET_VALUES: [
                    ("context.inputs:domain_id", 30),

                    ("computeOdom.inputs:chassisPrim", self.prim_path + '/base_link'),

                    ("publishRawTF.inputs:childFrameId", 'base_link'),
                    ("publishRawTF.inputs:topicName", f"/{self.name}/tf"),
                    ("publishRawTF.inputs:parentFrameId", 'odom'),

                    ("publishOdom.inputs:odomFrameId", 'odom'),
                    ("publishOdom.inputs:chassisFrameId", "base_link"),
                    ("publishOdom.inputs:topicName", f"/{self.name}/odom"),

                    ("publishTF.inputs:targetPrims", self.prim_path + '/base_link'),
                    ("publishTF.inputs:parentPrim", self.prim_path + '/base_link'),
                    ("publishTF.inputs:topicName", f"/{self.name}/tf"),

                    ("publishRawTF2.inputs:childFrameId", 'odom'),
                    ("publishRawTF2.inputs:topicName", f"/{self.name}/tf"),
                    ("publishRawTF2.inputs:parentFrameId", 'world'),
                ],
                og.Controller.Keys.CONNECT: [
                    ("onPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                    ("onPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                    ("onPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                    ("onPlaybackTick.outputs:tick", "publishRawTF2.inputs:execIn"),
                    ("onPlaybackTick.outputs:tick", "publishTF.inputs:execIn"),

                    ("readSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                    ("readSimTime.outputs:simulationTime", "publishRawTF.inputs:timeStamp"),
                    ("readSimTime.outputs:simulationTime", "publishRawTF2.inputs:timeStamp"),
                    ("readSimTime.outputs:simulationTime", "publishTF.inputs:timeStamp"),

                    ("context.outputs:context", "publishOdom.inputs:context"),
                    ("context.outputs:context", "publishRawTF.inputs:context"),
                    ("context.outputs:context", "publishRawTF2.inputs:context"),
                    ("context.outputs:context", "publishTF.inputs:context"),

                    ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                    ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                    ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                    ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                    ("computeOdom.outputs:orientation", "publishRawTF.inputs:rotation"),
                    ("computeOdom.outputs:position", "publishRawTF.inputs:translation"),

                ],
            }
        )
