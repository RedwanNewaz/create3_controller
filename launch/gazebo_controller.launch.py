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
    pkg_create3_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')



    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion
    # --ros-args __ns:=/ac31
    # ac31 autonomous create3 robot 1
    # create3_simple_controller = Node(
    #     package='create3_controller',
    #     executable='gazebo_controller',
    #     namespace='ac31',
    #     name='gazebo_simple_controller_1',
    #     # parameters=[
    #     #     {'sensor' : 'fusion',
    #     #      'robotTag': 'tag36h11:7',
    #     #      'logOutput' : "/var/tmp"
    #     #      }
    #     # ],
    #     # disable this line if not using rviz to send goal 
    #     # remappings=[
    #     #     ('/ac31/goal_pose', '/goal_pose')
    #     # ],
    #     output='screen',
    # )

    create3_simple_controller = Node(
        package='create3_controller',
        executable='gazebo_controller',
        name='gazebo_simple_controller230',
   
        # disable this line if not using rviz to send goal 
        remappings=[
            ('odom', '/ac31/odom'),
            ('cmd_vel', '/ac31/cmd_vel'),
        ],
        output='screen',
    )
    

    ld = LaunchDescription()
    ld.add_action(create3_simple_controller)


    return ld

