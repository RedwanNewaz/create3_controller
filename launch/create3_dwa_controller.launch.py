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

def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_controller")

    # ros2 launch create3_controller create3_apriltag.launch.py
    create3_apriltag = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_apriltag.launch.py'])]))

    create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_joystick.py'])]))
    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion
    create3_dwa_controller = Node(
        package='create3_controller',
        executable='dwa_controller',
        name='create3_dwa_controller',
        # arguments=['--ros-args',
        #            '-p',
        #            'sensor:=fusion',
        #            '-p',
        #            'control:=/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml'
        #            ],
        parameters=[
            {'sensor' : 'fusion',
             'control' : os.path.join(current_pkg_dir, 'config/dwa_param.yaml'),
             'robotTag': 'tag36h11:7',
             'logOutput' : "/var/tmp"
             }
        ],
        output='screen',
    )

    create3_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='create3_rviz2',
        arguments=['-d', os.path.join(current_pkg_dir, 'config/create3_state.rviz')],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(create3_apriltag)
    ld.add_action(create3_joystick)
    ld.add_action(create3_dwa_controller)
    ld.add_action(create3_rviz)
    return ld

