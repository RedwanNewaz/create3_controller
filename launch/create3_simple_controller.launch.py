from launch import  LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import sys
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

TAG_MAP = {
    'ac31' : 'tag36h11:7',
    'ac32' : 'tag36h11:32'
}

def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_controller")
    namespace = LaunchConfiguration('namespace')

    # ros2 launch create3_controller create3_apriltag.launch.py

    create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_joystick.py'])]),
        launch_arguments={'namespace': namespace}.items()
        )

    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion
    robotTag = 'tag36h11:32'
    nameFilter = lambda x: x.split("=")[-1]
    robotName = ""
    if len(sys.argv) > 4:
        robotName = "/" + nameFilter(sys.argv[4])
        robotTag = TAG_MAP.get(robotName, robotTag)


    create3_state_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_ekf.py'])]),
        launch_arguments={'namespace': namespace}.items()
    )


    # ac31 autonomous create3 robot 1
    create3_simple_controller = Node(
        package='create3_controller',
        executable='SimpleController_node',
        namespace=namespace,
        name='create3_simple_controller',
        parameters=[
            {'sensor' : 'odom',
             'robotTag': robotTag,
             'logOutput' : "/var/tmp",
             'tagTopic' : "/apriltag/detections",
             'safetyOverlook' : False
             }
        ],
        remappings=[
            # This maps the '/ekf/fusion' state topic for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("%s/ekf/fusion" % robotName, "%s/ekf/odom" %robotName)
        ],
        output='screen',
    )


    # ros2 run create3_controller waypoint_action_server
    create3_waypoint_controller = Node(
        package='create3_controller',
        executable='waypoint_action_server',
        namespace=namespace,
        name='create3_waypoint_action_server',
        parameters=[
            {'robotTopic' : '%s/ekf/odom' %robotName }
        ],
        output='screen'
    )



    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_joystick)
    ld.add_action(create3_state_node)
    ld.add_action(create3_simple_controller)
    ld.add_action(create3_waypoint_controller)

    return ld

