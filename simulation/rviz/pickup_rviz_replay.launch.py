from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = Path(get_package_share_directory("rmcs_energy_trace_v2"))
    urdf_path = LaunchConfiguration("urdf").perform(context)
    if not urdf_path:
        urdf_path = str(
            Path(get_package_share_directory("arm_description")) / "urdf/arm_description.urdf"
        )
    description = Path(urdf_path).expanduser().read_text()
    plan = str(Path(LaunchConfiguration("plan").perform(context)).expanduser().resolve())
    scene = LaunchConfiguration("scene").perform(context)
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="pickup",
            parameters=[
                {
                    "robot_description": description,
                    "frame_prefix": "pickup/",
                    "publish_frequency": 120.0,
                }
            ],
            remappings=remappings,
            output="log",
        ),
        Node(
            package="rmcs_energy_trace_v2",
            executable="pickup_rviz_replay",
            namespace="pickup",
            parameters=[
                {
                    "robot_description": description,
                    "plan_path": plan,
                    "scene_path": scene or str(share / "simulation/scene.json"),
                    "speed": float(LaunchConfiguration("speed").perform(context)),
                    "loop": LaunchConfiguration("loop").perform(context).lower() == "true",
                    "autoplay": LaunchConfiguration("autoplay").perform(context).lower() == "true",
                }
            ],
            remappings=remappings,
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace="pickup",
            arguments=["-d", str(share / "config/pickup_rviz_replay.rviz")],
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("plan", description="Path to the pickup plan JSON file"),
            DeclareLaunchArgument("urdf", default_value=""),
            DeclareLaunchArgument("scene", default_value=""),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("autoplay", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
