import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys
import yaml
import argparse



def get_container(name, cam_param, cfg_36h11):
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

ARGUMENTS = [
    DeclareLaunchArgument('node_name', default_value='',
                          description='create3_controller apriltag'),
]


def generate_launch_description():
    ld = LaunchDescription()
    current_pkg_dir = get_package_share_directory("create3_controller")
    assert len(sys.argv) > 3, "needs argument ['logitec', 'nexigo']"
    node_name = str(sys.argv[4]).split('=')[-1]

    print(node_name)
    

    # detect all 36h11 tags
    cfg_36h11_path = os.path.join(current_pkg_dir, 'config', 'cfg_36h11.yaml')
    with open(cfg_36h11_path) as file:
        cfg_36h11 = yaml.safe_load(file)
    

    # get joint camera config path
    cfg_joint_cams_path = os.path.join(current_pkg_dir, 'config', 'cfg_joint_cameras.yaml') 
    with open(cfg_joint_cams_path) as yfile:
        cfg_joint_cams = yaml.safe_load(yfile)
    
    nexigo_cam = cfg_joint_cams["nexigo_cam"]
    nexigo_cam['camera_info_url'] = nexigo_cam['camera_info_url'].format(current_pkg_dir)

    logitec_cam = cfg_joint_cams["logitec_cam"]
    logitec_cam['camera_info_url'] = logitec_cam['camera_info_url'].format(current_pkg_dir)

    
    if node_name == "nexigo":
        container = get_container(node_name, nexigo_cam, cfg_36h11)
        ld.add_action(container)
    elif node_name == "logitec":
        container = get_container(node_name, logitec_cam, cfg_36h11)
        ld.add_action(container)



    return ld
