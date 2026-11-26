from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_adapter(context):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in ("formal", "isolated_test"):
        raise ValueError("profile must be formal or isolated_test")
    config = LaunchConfiguration("config").perform(context)
    if not config:
        config = join(get_package_share_directory("culvert_livo_adapter"),
                      "config", profile + ".yaml")
    return [Node(package="culvert_livo_adapter", executable="culvert_livo_adapter_node",
                 name="culvert_livo_adapter", output="screen",
                 parameters=[config, {"profile": profile,
                                      "use_sim_time": ParameterValue(
                                          LaunchConfiguration("use_sim_time"), value_type=bool)}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="formal"),
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_adapter),
    ])
