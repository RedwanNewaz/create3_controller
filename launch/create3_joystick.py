import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_node = Node(
        name='joy_node',
        package="joy",
        namespace='ac31',
        executable="joy_node"
    )

    teleop_node = Node(
        name = 'teleop_node',
        namespace='ac31',
        package = "teleop_twist_joy",
        executable="teleop_node"
    )

    return launch.LaunchDescription([joy_node, teleop_node])