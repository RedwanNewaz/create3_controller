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

cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True,
    "detector": {"threads" : 12, "sharpening": 0.25},
    "tag": {"ids" : [7, 32]}
}

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    create3_controller_dir = get_package_share_directory('create3_controller')
    parameters_file_path = os.path.join(create3_controller_dir, 'config', 'dual_ekf.yaml')


    # tag_node = Node(
    #     package='apriltag_ros',
    #     executable='apriltag_node',
    #     namespace=namespace,
    #     name='create3_state_estimator',
    #     parameters=[cfg_36h11],
    #     remappings=[
    #         # This maps the 'raw' images for simplicity of demonstration.
    #         # In practice, this will have to be the rectified 'rect' images.
    #         ("/apriltag/image_rect", "/camera/image_raw"),
    #         ("/apriltag/camera_info", "/camera/camera_info"),
    #     ],
    #     output='screen',
    # )


    # tag_node = ComposableNode(
    #     name='apriltag_36h11',
    #     namespace='apriltag',
    #     package='apriltag_ros', plugin='AprilTagNode',
    #     remappings=[
    #         # This maps the 'raw' images for simplicity of demonstration.
    #         # In practice, this will have to be the rectified 'rect' images.
    #         ("/apriltag/image_rect", "/camera/image_raw"),
    #         ("/apriltag/camera_info", "/camera/camera_info"),
    #     ],
    #     parameters=[cfg_36h11],
    #     extra_arguments=[{'use_intra_process_comms': True}],
    # )
    #
    #
    # container = ComposableNodeContainer(
    #     name='tag_container',
    #     namespace='apriltag',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[tag_node],
    #     output='screen'
    # )


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

    static_transform_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='my_static_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
    )

    ld = launch.LaunchDescription(ARGUMENTS)
    ld.add_action(static_transform_node)
    ld.add_action(create3_state_estimator)
    ld.add_action(ekf_odom)
    ld.add_action(ekf_apriltag)
    ld.add_action(ekf_filter_node_fusion)


    return ld