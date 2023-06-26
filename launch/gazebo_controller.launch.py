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
    pkg_create3_gazebo_bringup = get_package_share_directory('irobot_create_gazebo_bringup')
    gazebo_sim_launch_file = PathJoinSubstitution(
        [pkg_create3_gazebo_bringup, 'launch', 'create3_gazebo.launch.py'])
    create3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_sim_launch_file]),
        launch_arguments={'use_gazebo_gui': 'false'}.items()
    )



    create3_controller_gui = Node(
        package='create3_controller',
        executable='create3_controller_gui',
        name='create3_controller_gui'
    )

    create3_gazebo_simple_controller = Node(
        package='create3_controller',
        executable='gazebo_controller',
        name='create3_gazebo_simple_controller'
    )

    # ros2 run create3_controller waypoint_action_server
    create3_waypoint_controller = Node(
        package='create3_controller',
        executable='waypoint_action_server',
        name='create3_waypoint_action_server',
        output='screen'
    )

    #create3 map server
    create3_map_server = Node(
        package='create3_controller',
        executable='dynamic_map_server',
        name='create3_map_server',
        output='screen'
    )

    # static transform odom to map
    odom_to_map = Node(package = "tf2_ros",
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "odom", "map"])
    

    ld = LaunchDescription()
    ld.add_action(create3_gazebo)
    ld.add_action(create3_controller_gui)
    ld.add_action(create3_gazebo_simple_controller)
    ld.add_action(create3_waypoint_controller)
    ld.add_action(create3_map_server)
    ld.add_action(odom_to_map)


    return ld

