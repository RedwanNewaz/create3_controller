from launch import  LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution
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

conf_sensor_fusion = {
    "tag_id": 7,
    "tag_size": 0.2,
    'logitec_pmat': [933.4418334960938, 0.0, 978.0901233670083, 0.0, 0.0, 995.1202392578125, 490.9420947208673, 0.0, 0.0, 0.0, 1.0, 0.0],
    'nexigo_pmat': [863.1061401367188, 0.0, 946.3947846149531, 0.0, 0.0, 903.219482421875, 411.1189551965581, 0.0, 0.0, 0.0, 1.0, 0.0],
    'logitec_to_map': [-0.232, 1.258, 3.098, 0.996, -0.013, -0.026, 0.073],
    'nexigo_to_map': [0.259, 1.737, 3.070, -0.014, 0.970, 0.226, 0.080],
    'ctrv_mtx': [0.005,  0.0, 0.0, 0.005],
    'lidar_mtx': [0.0225, 0.0, 0.0, 0.0225],
    'radar_mtx': [2.050,  0.000,  0.00, 0.000,  2.050,  0.00, 0.000,  0.000,  0.09],
    'sensor_fusion': 1
}

def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_controller")
    sensor_fusion_dir = get_package_share_directory("sensor_fusion")
    namespace = LaunchConfiguration('namespace')


    # ros2 launch create3_controller create3_apriltag.launch.py
    create3_apriltag = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'airlab_cameras.launch.py'])]))

    create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_joystick.py'])]),
        launch_arguments={'namespace': namespace}.items()
        )

    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion
    robotTag = 'tag36h11:32'
    nameFilter = lambda x: x.split("=")[-1]
    robotName = ""
    if len(sys.argv) > 4:
        robotName = nameFilter(sys.argv[4])
        robotTag = TAG_MAP.get(robotName, robotTag)


    # get path to params file
    params_path = os.path.join(
        sensor_fusion_dir,
        'config',
        '{}_params.yaml'.format(robotName)
    )

    conf_sensor_fusion['tag_id'] = 32 if robotName == 'ac32' else 7

    create3_state_node = Node(
        package='sensor_fusion', executable='sensor_fusion_node', output='screen',
        name="sensor_fusion_node",
        namespace=namespace,
        parameters=[conf_sensor_fusion]
    )

    state_topic = "/%s/apriltag/state" % robotName





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
            ("/%s/ekf/fusion" % robotName, state_topic)
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
            {'robotTopic' : state_topic }
        ],
        output='screen'
    )

    create3_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='create3_rviz2',
        arguments=['-d', os.path.join(current_pkg_dir, 'config/multicam.rviz')],
        output='screen',
    )



    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_apriltag)
    ld.add_action(create3_joystick)
    ld.add_action(create3_state_node)
    ld.add_action(create3_simple_controller)
    ld.add_action(create3_waypoint_controller)
    ld.add_action(create3_rviz)

    return ld

