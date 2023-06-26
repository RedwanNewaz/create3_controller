import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

TAG_MAP = {
    '/ac31' : 'tag36h11:7',
    '/ac32' : 'tag36h11:32'
}
def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    # ros2 run create3_controller waypoint_action_server
    create3_waypoint_controller = Node(
        package='create3_controller',
        executable='waypoint_action_server',
        namespace=namespace,
        name='create3_waypoint_action_server',
        parameters=[
            {'robotTopic' : 'odom/filtered'}
        ],
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_waypoint_controller)


    return ld

