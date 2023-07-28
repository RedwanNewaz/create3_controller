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


    robotName = 'ac32'
    if len(sys.argv) > 4:
        robotName = sys.argv[4].split("=")[-1]

    print(TAG_MAP.get(robotName))

    create3_state_estimator = Node(
        package="create3_controller",
        executable="create3_state_estimator",
        namespace=namespace,
        name='create3_state_estimator',
        parameters=[
            {'sensor' : 'apriltag',
             'robotTag': TAG_MAP.get(robotName, 'tag36h11:32'),
             'logOutput' : "/home/roboticslab/colcon_ws/src/create3_controller/launch/build"
             }]
    )

    tf_nexigo_map = [0.259, 1.737, 3.070, -0.014, 0.970, 0.226, 0.080]
    nexigo_to_logitec = [0.274, 3.869 - 0.25, 0.484, -0.059, -0.153, 0.986, -0.026]
    tf_nexigo_map_arg = list(map(str, tf_nexigo_map)) + ["nexigo_cam", "map"]
    tf_nexigo_logitec_arg = list(map(str, nexigo_to_logitec)) + ["nexigo_cam", "logitec_cam"]
    nexigo_to_map = Node(package = "tf2_ros",
                         name = "tf_nexigo_map",
                         executable = "static_transform_publisher",
                         arguments = tf_nexigo_map_arg)

    nexigo_to_logitec = Node(package = "tf2_ros",
                             name="tf_logitec_nexigo",
                             executable = "static_transform_publisher",
                             arguments = tf_nexigo_logitec_arg)


    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(create3_state_estimator)
    # static transform
    ld.add_action(nexigo_to_logitec)
    ld.add_action(nexigo_to_map)

    return ld

