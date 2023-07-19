import sys
from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

TAG_MAP = {
    'ac31' : 'tag36h11:7',
    'ac32' : 'tag36h11:32'
}

def getRobotName():
    robotName = ""
    nameFilter = lambda x: x.split("=")[-1]
    if len(sys.argv) > 4:
        robotName =  nameFilter(sys.argv[4])
    return robotName

def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_controller")
    namespace = LaunchConfiguration('namespace')

    # ros2 launch create3_controller create3_apriltag.launch.py
    create3_apriltag = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_apriltag.launch.py'])]))

    # create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
    #     [current_pkg_dir, 'launch', 'create3_joystick.py'])]))
    create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_joystick.py'])]),
        launch_arguments={'namespace': namespace}.items()
        )
    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion


    create3_dwa_controller = Node(
        package='create3_controller',
        executable='dwa_controller',
        namespace=namespace,
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
             'robotTag': TAG_MAP.get(getRobotName(), 'tag36h11:7'),
             'logOutput' : "/var/tmp"
             }
        ],
        output='screen',
          # disable this line if not using rviz to send goal 
        # remappings=[
        #     ('/%s/goal_pose'%namespace, '/goal_pose')
        # ]
    )

    create3_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='create3_rviz2',
        arguments=['-d', os.path.join(current_pkg_dir, 'config/create3_state.rviz')],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(create3_apriltag)
    ld.add_action(create3_joystick)
    ld.add_action(create3_dwa_controller)
    ld.add_action(create3_rviz)
    return ld

