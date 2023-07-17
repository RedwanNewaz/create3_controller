from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_controller")
    create3_controller_dir = get_package_share_directory('create3_controller')

    state_node = Node(
        name='StateEstimator',
        package='create3_controller', executable='StateEstimator_node',
        namespace="",
        parameters=[  {'sensor' : 'sim',
                       'robotTag': 'tag36h11:7',
                       'logOutput' : "/var/tmp",
                       'tagTopic' : "/apriltag/detections"
                       }]
    )

    controller_node = Node(
        name='SimpleController',
        package='create3_controller', executable='SimpleController_node',
        namespace="",
        remappings=[
            # This maps the '/ekf/fusion' state topic for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/ekf/fusion", "/ekf/odom")
        ],

    )

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='my_static_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'map']
    )


    create3_waypoint_controller = Node(
        package='create3_controller',
        executable='waypoint_action_server',
        name='create3_waypoint_action_server',
        parameters=[
            {'robotTopic' : '/ekf/odom' }
        ],
        output='screen'
    )

    #create3 map server
    create3_map_server = Node(
        package='create3_controller',
        executable='dynamic_map_server',
        name='create3_map_server',
        output='screen'
    )


    #create3 viewer
    create3_viewer = Node(
        package='create3_controller',
        executable='create3_view_node',
        name='create3_view_node'
    )



    ld = LaunchDescription()
    ld.add_action(state_node)
    ld.add_action(controller_node)
    ld.add_action(static_transform_node)
    ld.add_action(create3_viewer)
    ld.add_action(create3_waypoint_controller)
    ld.add_action(create3_map_server)


    return ld

