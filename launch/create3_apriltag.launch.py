import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

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


def generate_launch_description():
    current_pkg_dir = get_package_share_directory("create3_controller")
    #default_ros_cam_dir = "file:///home/robo/.ros/camera_info/head_camera_nexigo_1920.yaml"
    nexigo_cam = {
        "camera_name" : 'nexigo_cam',
        "camera_info_url": "file://{}/config/head_camera_nexigo_1920.yaml".format(current_pkg_dir),
        "framerate" : 60.0,
        "frame_id" : "camera",
        "image_height"  : 1080,
        "image_width"   : 1920,
        "io_method"     : "mmap",
        "pixel_format"  : "mjpeg",
        # "color_format"  : "yuv422p",
        "video_device"  : "/dev/video0"
    }

    # configure apriltag
    # cfg_36h11 = os.path.join(current_pkg_dir, 'config', 'cfg_36h11.yaml')

    cam_node = ComposableNode(
        namespace='camera',
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        parameters=[nexigo_cam],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    tag_node = ComposableNode(
        name='apriltag_36h11',
        namespace='apriltag',
        package='apriltag_ros', plugin='AprilTagNode',
        remappings=[
            # This maps the 'raw' images for simplicity of demonstration.
            # In practice, this will have to be the rectified 'rect' images.
            ("/apriltag/image_rect", "/camera/image_raw"),
            ("/apriltag/camera_info", "/camera/camera_info"),
        ],
        parameters=[cfg_36h11],
        extra_arguments=[{'use_intra_process_comms': True}],
    )


    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam_node, tag_node],
        output='screen'
    )


    return launch.LaunchDescription([container])
