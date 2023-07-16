from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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
    current_pkg_dir = get_package_share_directory("create3_controller")
    create3_controller_dir = get_package_share_directory('create3_controller')
    parameters_file_path = os.path.join(create3_controller_dir, 'config', 'dual_ekf.yaml')

    tag_node = ComposableNode(
        name='apriltag_36h11',
        namespace='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag/image_rect", "/camera/image_raw"),
            ("/apriltag/camera_info", "/camera/camera_info"),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    state_node = Node(
        name='StateEstimator',
        package='create3_controller', executable='StateEstimator_node',
        namespace="",
        parameters=[  {'sensor' : 'fusion',
                       'robotTag': 'tag36h11:7',
                       'logOutput' : "/var/tmp",
                       'tagTopic' : "/apriltag/detections"
                       }]
    )

    state_node_compose = ComposableNode(
        name='StateEstimator',
        package='create3_controller', plugin='model::JointStateEstimator',
        namespace="",
        parameters=[  {'sensor' : 'fusion',
                       'robotTag': 'tag36h11:7',
                       'logOutput' : "/var/tmp",
                       'tagTopic' : "/apriltag/detections"
                       }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[tag_node],
        output='screen'
    )

    create3_viewer = Node(
        package='create3_controller',
        executable='create3_view_node',
        name='create3_view_node'
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

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='my_static_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
    )

    ld = LaunchDescription()
    ld.add_action(container)
    ld.add_action(state_node)
    ld.add_action(ekf_odom)
    ld.add_action(ekf_apriltag)
    ld.add_action(ekf_filter_node_fusion)
    ld.add_action(static_transform_node)
    ld.add_action(create3_viewer)
    # ld.add_action(create3_viewer)
    return ld

