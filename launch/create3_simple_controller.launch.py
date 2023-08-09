import argparse
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.2, # 0.162
    "max_hamming": 0,
    "z_up": True,
    "detector": {"threads" : 12, "sharpening": 0.25},
    "tag": {"ids" : [7, 32]}
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


    create3_joystick = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution(
        [current_pkg_dir, 'launch', 'create3_joystick.py'])]),
        launch_arguments={'namespace': namespace}.items()
    )

    # --ros-args -p control:="/home/roboticslab/colcon_ws/src/create3_controller/config/dwa_param.yaml" -p sensor:=fusion

    nameFilter = lambda x: x.split("=")[-1]
    robotName = ""
    if len(sys.argv) > 4:
        robotName = nameFilter(sys.argv[4])



    conf_sensor_fusion['tag_id'] = 32 if robotName == 'ac32' else 7
    # get path to params file
    nexigo_cam = {
        "camera_name" : 'nexigo_cam',
        "camera_info_url": "file://{}/config/head_camera_nexigo_1920.yaml".format(current_pkg_dir),
        "framerate" : 60.0,
        "frame_id" : "nexigo_cam",
        "image_height"  : 1080,
        "image_width"   : 1920,
        "io_method"     : "mmap",
        "pixel_format"  : "mjpeg",
        # "color_format"  : "yuv422p",
        "video_device"  : "/dev/video0"
    }

    logitec_cam = {
        "camera_name" : 'logitec_cam',
        "camera_info_url": "file://{}/config/head_camera_logitec_1920.yaml".format(current_pkg_dir),
        "framerate" : 60.0,
        "frame_id" : "logitec_cam",
        "image_height"  : 1080,
        "image_width"   : 1920,
        "io_method"     : "mmap",
        "pixel_format"  : "mjpeg",
        # "color_format"  : "yuv422p",
        "video_device"  : "/dev/video2"
    }

    nexigo_cam_node = ComposableNode(
        namespace="nexigo",
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        parameters=[nexigo_cam],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    logitec_cam_node = ComposableNode(
        namespace="logitec",
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        parameters=[logitec_cam],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    name = "nexigo"
    nexigo_apriltag_node = ComposableNode(
        name='apriltag_36h11_%s' % name,
        namespace='apriltag_%s' % name,
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag_%s/image_rect" % name, "/%s/image_raw" % name),
            ("/apriltag_%s/camera_info" % name, "/%s/camera_info" % name),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    name = "logitec"
    logitec_apriltag_node = ComposableNode(
        name='apriltag_36h11_%s' % name,
        namespace='apriltag_%s' % name,
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag_%s/image_rect" % name, "/%s/image_raw" % name),
            ("/apriltag_%s/camera_info" % name, "/%s/camera_info" % name),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    sensor_fusion_node = ComposableNode(
        namespace=namespace,
        package='sensor_fusion', plugin='airlab::apriltag_fusion',
        parameters=[conf_sensor_fusion],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    state_topic = "/%s/apriltag/state" % robotName

    # create3_simple_controller = ComposableNode(
    #     namespace=namespace,
    #     package='create3_controller', plugin='controller::UnicycleController',
    #     parameters=[{'safetyOverlook' : False}],
    #     remappings=[("/%s/ekf/fusion" % robotName, state_topic)],
    #     extra_arguments=[{'use_intra_process_comms': True}],
    # )

    create3_waypoint_controller = ComposableNode(
        namespace=namespace,
        package='create3_controller', plugin='navigation::WaypointController',
        parameters=[{'robotTopic' : state_topic}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    # create3_simple_controller = ComposableNode(
    #     namespace=namespace,
    #     package='create3_controller', plugin='controller::UnicycleController',
    #     parameters=[{'safetyOverlook' : False}],
    #     remappings=[("/%s/ekf/fusion" % robotName, state_topic)],
    #     extra_arguments=[{'use_intra_process_comms': True}],
    # )


    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[nexigo_cam_node, logitec_cam_node,
                                      nexigo_apriltag_node, logitec_apriltag_node,
                                      sensor_fusion_node,  create3_waypoint_controller],
        output='screen'
    )

    # ac31 autonomous create3 robot 1
    create3_simple_controller = Node(
        package='create3_controller',
        executable='SimpleController_node',
        namespace=namespace,
        name='create3_simple_controller',
        parameters=[
            {'safetyOverlook' : False}
        ],
        remappings=[
            # This maps the '/ekf/fusion' state topic for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/%s/ekf/fusion" % robotName, state_topic)
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



    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(container)
    # ld.add_action(create3_simple_controller)
    ld.add_action(create3_joystick)
    ld.add_action(create3_rviz)

    return ld

