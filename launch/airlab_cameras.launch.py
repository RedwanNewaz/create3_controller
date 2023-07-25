
import argparse
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import sys

from ament_index_python.packages import get_package_share_directory

# detect all 36h11 tags
cfg_36h11 = {
    "image_transport": "raw",
    "family": "36h11",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True,
    "detector": {"threads" : 12, "sharpening": 0.25},
    "tag": {"ids" : [7, 32]}
}


def get_nodes(ld, nexigo_cam, logitec_cam):
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="nexigo_cam_node",
        namespace="nexigo",
        parameters=[nexigo_cam]
    ))
    name = "nexigo"
    ld.add_action(Node(
        package="apriltag_ros", executable="apriltag_node", output="screen",
        name="apriltag_%s_node" %name,
        namespace="apriltag_%s" %name,
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag_%s/image_rect" % name, "/%s/image_raw" % name),
            ("/apriltag_%s/camera_info" % name, "/%s/camera_info" % name),
        ],
        parameters=[cfg_36h11],
    ))

    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="logitec_cam_node",
        namespace="logitec",
        parameters=[logitec_cam]
    ))

    name = "logitec"
    ld.add_action(Node(
        package="apriltag_ros", executable="apriltag_node", output="screen",
        name="apriltag_%s_node" %name,
        namespace="apriltag_%s" %name,
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag_%s/image_rect" % name, "/%s/image_raw" % name),
            ("/apriltag_%s/camera_info" % name, "/%s/camera_info" % name),
        ],
        parameters=[cfg_36h11],
    ))


def get_container(name, cam_param):
    assert name in ["logitec", "nexigo"]

    cam_node = ComposableNode(
        namespace=name,
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        parameters=[cam_param],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    tag_node = ComposableNode(
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


    container = ComposableNodeContainer(
        name='%s_tag_container' % name,
        namespace='apriltag_%s' % name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam_node, tag_node],
        output='screen'
    )

    return container

def generate_launch_description():
    ld = LaunchDescription()

    # parser = argparse.ArgumentParser(description='usb_cam demo')
    # parser.add_argument('-n', '--node-name', dest='node_name', type=str,
    #                     choices=['nexigo', 'logitec'],
    #                     help='name for device', default='logitec')
    #
    # args, unknown = parser.parse_known_args(sys.argv[4:])

    current_pkg_dir = get_package_share_directory('create3_controller')

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
        "video_device"  : "/dev/video2"
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
        "video_device"  : "/dev/video0"
    }


    nexigo_container = get_container("nexigo", nexigo_cam)
    logitec_container = get_container("logitec", logitec_cam)


    # static transform
    tf_logitec_map = [0.45433333,  0.772,  3.0265,  0.95433333,  0.00733333,  0.00333333, -0.29733333]
    tf_nexigo_map = [-0.47433333,  0.89816667,  2.41483333,  0.01066667,  0.98533333, -0.16933333, 0.01066667]
    tf_map_nexigo = [-0.430, -0.029, 2.584, 0.011, 0.985, -0.169, -0.011]
    nexigo_to_logitec = [0.099, 2.427, -0.291, -0.016, -0.132, 0.991, 0.011]
    logitec_to_nexigo = [0.026, 2.267, 0.919, -0.016, -0.132, 0.991, -0.011]

    tf_logitec_map_arg = list(map(str, tf_logitec_map)) + ["logitec_cam", "map"]
    tf_nexigo_map_arg = list(map(str, tf_nexigo_map)) + ["nexigo_cam", "map"]
    tf_logitec_nexigo_arg = list(map(str, logitec_to_nexigo)) + ["logitec_cam", "nexigo_cam"]
    tf_nexigo_logitec_arg = list(map(str, nexigo_to_logitec)) + ["nexigo_cam", "logitec_cam"]



    # nexigo_to_map = Node(package = "tf2_ros",
    #                      name = "tf_nexigo_map",
    #                      executable = "static_transform_publisher",
    #                      arguments = tf_nexigo_map_arg)


    logitec_to_map = Node(package = "tf2_ros",
                          name="tf_logitec_map",
                          executable = "static_transform_publisher",
                          arguments = tf_logitec_map_arg)
    
    logitec_to_nexigo = Node(package = "tf2_ros",
                          name="tf_logitec_nexigo",
                          executable = "static_transform_publisher",
                          arguments = tf_logitec_nexigo_arg)
    
    # nexigo_to_logitec = Node(package = "tf2_ros",
    #                       name="tf_nexigo_logitec",
    #                       executable = "static_transform_publisher",
    #                       arguments = tf_nexigo_logitec_arg)







    ld.add_action(nexigo_container)
    ld.add_action(logitec_container)
    ld.add_action(logitec_to_nexigo)
    ld.add_action(logitec_to_map)
    # ld.add_action(nexigo_to_logitec)


    return ld
