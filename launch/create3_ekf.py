import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import os
import sys
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]
TAG_MAP = {
    'ac31' : 'tag36h11:7',
    'ac32' : 'tag36h11:32'
}

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    create3_controller_dir = get_package_share_directory('create3_controller')
    parameters_file_path = os.path.join(create3_controller_dir, 'config', 'dual_ekf.yaml')

    robotTag = 'tag36h11:32'
    nameFilter = lambda x: x.split("=")[-1]
    robotName = ""
    if len(sys.argv) > 4:
        robotName = nameFilter(sys.argv[4])
        robotTag = TAG_MAP.get(robotName, robotTag)

    print("create3_ekf.py file parsing args")
    print(robotName, robotTag)

    create3_state_estimator = Node(
        name='StateEstimator',
        package='create3_controller', executable='StateEstimator_node',
        namespace=namespace,
        parameters=[  {'sensor' : 'fusion',
                       'robotTag': robotTag,
                       'logOutput' : "/var/tmp",
                       'tagTopic' : "/apriltag/detections"
                       }],
        output='screen'
    )


    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        namespace=namespace,
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('%s/odometry/filtered' % robotName, '%s/ekf/odom/filtered' % robotName)]
    )
    ekf_apriltag = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='ekf_filter_node_apriltag',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('%s/odometry/filtered' %robotName, '%s/ekf/apriltag/filtered' % robotName)]
    )

    ekf_filter_node_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=namespace,
        name='ekf_filter_node_fusion',
        output='screen',
        parameters=[parameters_file_path],
        remappings=[('%s/odometry/filtered' % robotName, '%s/ekf/fusion' % robotName)]
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