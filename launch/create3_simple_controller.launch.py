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
    create3_apriltag = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'airlab_cameras.launch.py'])]))

    create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_joystick.py'])]),
        launch_arguments={'namespace': namespace}.items()
    )

    robotName = 'ac32'
    if len(sys.argv) > 4:
        robotName = sys.argv[4].split("=")[-1]

    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion
    print(namespace, TAG_MAP.get(robotName))
    # ac31 autonomous create3 robot 1
    create3_simple_controller = Node(
        package='create3_controller',
        executable='simple_controller',
        namespace=namespace,
        name='create3_simple_controller',
        parameters=[
            {'sensor' : 'apriltag',
             'robotTag': TAG_MAP.get(robotName, 'tag36h11:32'),
             'logOutput' : "/home/roboticslab/colcon_ws/src/create3_controller/launch/build"
             }
        ],
        output='screen',
    )

    create3_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='create3_rviz2',
        arguments=['-d', os.path.join(current_pkg_dir, 'config/multicam.rviz')],
        output='screen',
    )

    # ros2 run create3_controller waypoint_action_server
    create3_waypoint_controller = Node(
        package='create3_controller',
        executable='waypoint_action_server',
        namespace=namespace,
        name='create3_waypoint_action_server',
        parameters=[
            {'robotTopic' : 'ekf/odom' }
        ],
        output='screen'
    )

    #create3 map server
    map_parm = {
        'mapResolution': 0.0052,
        'mapOffsetX': 0.0,
        'mapOffsetY': 0.55 

    }
    create3_map_server = Node(
        package='create3_controller',
        executable='dynamic_map_server',
        name='create3_map_server',
        output='screen',
        parameters=[map_parm]
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(create3_apriltag)
    ld.add_action(create3_joystick)
    ld.add_action(create3_simple_controller)
    ld.add_action(create3_rviz)

    ld.add_action(create3_waypoint_controller)
    ld.add_action(create3_map_server)
    return ld

