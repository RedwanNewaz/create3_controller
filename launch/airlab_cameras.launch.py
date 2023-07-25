
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
    "size": 0.175, # 0.162
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


    # static transform nexigo

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


    # static transform logitec
    tf_logitec_map = [-0.232, 1.258, 3.098, 0.996, -0.013, -0.026, 0.073]
    logitec_to_nexigo = [0.463, 3.809 - 0.25, 0.750, -0.059, -0.153, 0.986, 0.026]
    tf_logitec_map_arg = list(map(str, tf_logitec_map)) + ["logitec_cam", "map"]
    tf_logitec_nexigo_arg = list(map(str, logitec_to_nexigo)) + ["logitec_cam", "nexigo_cam"]

    logitec_to_map = Node(package = "tf2_ros",
                          name="tf_logitec_map",
                          executable = "static_transform_publisher",
                          arguments = tf_logitec_map_arg)
    
    logitec_to_nexigo = Node(package = "tf2_ros",
                          name="tf_logitec_nexigo",
                          executable = "static_transform_publisher",
                          arguments = tf_logitec_nexigo_arg)


    ld.add_action(nexigo_container)
    ld.add_action(logitec_container)

    # static transform
    ld.add_action(nexigo_to_logitec)
    ld.add_action(nexigo_to_map)



    return ld
