import launch
from launch_ros.actions import ComposableNodeContainer

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    joy_node = Node(
        name='joy_node',
        package="joy",
        namespace=namespace,
        executable="joy_node"
    )

    teleop_node = Node(
        name = 'teleop_node',
        namespace=namespace,
        package = "teleop_twist_joy",
        executable="teleop_node"
    )

    ld = launch.LaunchDescription(ARGUMENTS)
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    return ld