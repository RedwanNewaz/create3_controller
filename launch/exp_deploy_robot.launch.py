from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription

def generate_launch_description():
    '''
    this launch file is responsible for
    (i) apriltag
    (ii) rviz
    :return:
    '''
    current_pkg_dir = get_package_share_directory("create3_controller")

    create3_apriltag = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_apriltag.launch.py'])]))

    # create3 interface
    create3_ui = Node(
        package='create3_controller',
        executable='create3_controller_gui',
        name='create3_gui_node'
    )

    #create3 viewer
    create3_viewer = Node(
        package='create3_controller',
        executable='create3_view_node',
        name='create3_view_node'
    )

    # launch rviz2
    create3_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='create3_rviz2',
        arguments=['-d', os.path.join(current_pkg_dir, 'config/create3_state_v2.rviz')],
        output='screen',
    )

    #create3 map server
    create3_map_server = Node(
        package='create3_controller',
        executable='dynamic_map_server',
        name='create3_map_server',
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(create3_ui)
    ld.add_action(create3_apriltag)
    ld.add_action(create3_viewer)
    ld.add_action(create3_rviz)
    ld.add_action(create3_map_server)

    return ld