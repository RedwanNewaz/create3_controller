
from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():
    map_saver_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=["/home/redwan/colcon_ws/src/create3_controller/config/map_server_params.yaml"])
    map_publisher = "/home/redwan/colcon_ws/src/create3_controller/config"

    # Parameters
    lifecycle_nodes = ['map_saver']
    use_sim_time = True
    autostart = True
    save_map_timeout = 2.0
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65
    #
    # # Nodes launching commands
    start_map_saver_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'save_map_timeout': save_map_timeout},
                    {'free_thresh_default': free_thresh_default},
                    {'occupied_thresh_default': occupied_thresh_default}])

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])



    ld = LaunchDescription()
    ld.add_action(map_saver_server_cmd)
    ld.add_action(map_publisher_cmd)


    # ld.add_action(map_saver_server_cmd)
    # ld.add_action(start_map_saver_server_cmd)
    # ld.add_action(start_lifecycle_manager_cmd)

    return ld