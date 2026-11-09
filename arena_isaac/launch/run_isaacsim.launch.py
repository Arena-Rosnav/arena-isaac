import launch
from arena_rclpy_mixins.launch_argument import LaunchArgument
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    ld = []
    LaunchArgument.auto_append(ld)

    logger = LaunchArgument(
        name='log_level',
        default_value='debug',
        description='Log level.',
    )

    headless = LaunchArgument(
        name='headless',
        default_value='False',
        description='Run Isaac Sim without its GUI.',
    )

    physics = LaunchArgument(
        name='physics',
        default_value='physx',
        choices=['physx', 'newton'],
        description='Physics engine of Isaac Sim.',
    )

    viewport = {
        key: LaunchArgument(name=f'viewport.{key}', default_value=default, description=description)
        for key, default, description in (
            ('preset', 'photoreal', 'Base render preset: photoreal | boring. GUI only.'),
            ('resolution', '', 'Viewport render size: WxH | dynamic. Empty keeps the preset value.'),
            ('scale', '', 'Viewport resolution scale factor. Empty keeps the preset value.'),
            ('dlss', '', 'DLSS mode: auto | quality | balanced | performance. Empty keeps the preset value.'),
            ('lighting', '', 'Lighting rig: lights_off | camera_light | stage_lights | colored_lights | default | grey_studio. Empty keeps the preset value.'),
            ('overlays', '', 'Viewport overlays to show, comma list of axis,grid,bbox or none. Empty keeps the preset value.'),
            ('ceilings', '', 'Room ceilings: auto | on | off. Empty is auto, hidden so the GUI looks into the rooms from above unless the world declares lights.'),
        )
    }

    run_isaacsim_path = ExecutableInPackage(
        executable='run_isaacsim',
        package='arena_isaac',
    )

    return LaunchDescription([
        *ld,
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([EnvironmentVariable('ISAAC_PATH'), 'python.sh']),
                run_isaacsim_path,
                '--log-level', logger.substitution,
                '--headless', headless.substitution,
                ['physics:=', physics.substitution],
                *([f'viewport.{key}:=', arg.substitution] for key, arg in viewport.items()),
            ],
            output='screen',
        ),
    ])
