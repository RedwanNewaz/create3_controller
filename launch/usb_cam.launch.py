
import argparse
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory



def get_container(name, cam_param, cfg_36h11):
    assert name in ["logitec", "nexigo"]

    # cam_param['camera_name'] = 'camera'

    cam_node = ComposableNode(
        namespace=name,
        package='usb_cam', plugin='usb_cam::UsbCamNode',
        parameters=[cam_param],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    container = ComposableNodeContainer(
        name='%s_tag_container' % name,
        namespace='apriltag_%s' % name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam_node],
        output='screen'
    )
   

    return container

def generate_launch_description():
    ld = LaunchDescription()
    current_pkg_dir = get_package_share_directory('create3_controller')
    current_pkg_dir = str(current_pkg_dir)

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

    nexigo_container = get_container("nexigo", nexigo_cam, cfg_36h11)
    logitec_container = get_container("logitec", logitec_cam, cfg_36h11)

    ld.add_action(nexigo_container)
    ld.add_action(logitec_container)




    return ld
