import launch
from launch_ros.actions import ComposableNodeContainer

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


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
    create3_controller_dir = get_package_share_directory('create3_controller')
    parameters_file_path = os.path.join(create3_controller_dir, 'config', 'dual_ekf.yaml')


    create3_state_estimator = Node(
        package='create3_controller',
        executable='create3_state_estimator',
        namespace=namespace,
        name='create3_state_estimator',
        parameters=[
            {'sensor' : 'fusion',
             'robotTag': TAG_MAP.get(namespace, 'tag36h11:7'),
             'logOutput' : "/var/tmp"
             }
        ],
        output='screen',
    )


    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'ekf/odom/filtered')]
    )
    ekf_apriltag = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_apriltag',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'ekf/apriltag/filtered')]
    )

    ekf_filter_node_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_fusion',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('odometry/filtered', 'ekf/fusion')]
    )


    ld = launch.LaunchDescription(ARGUMENTS)
    ld.add_action(create3_state_estimator)
    ld.add_action(ekf_odom)
    ld.add_action(ekf_apriltag)
    ld.add_action(ekf_filter_node_fusion)

    return ld