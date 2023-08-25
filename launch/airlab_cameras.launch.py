
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


    # static transform nexigo

    tf_nexigo_map = nexigo_cam["tf"]["map"]
    tf_nexigo_to_logitec = nexigo_cam["tf"]["logitec"]
    tf_nexigo_map_arg = list(map(str, tf_nexigo_map)) + ["nexigo_cam", "map"]
    tf_nexigo_logitec_arg = list(map(str, tf_nexigo_to_logitec)) + ["nexigo_cam", "logitec_cam"]
    
    nexigo_to_map = Node(package = "tf2_ros",
                         name = "tf_nexigo_map",
                         executable = "static_transform_publisher",
                         arguments = tf_nexigo_map_arg)

    nexigo_to_logitec = Node(package = "tf2_ros",
                             name="tf_logitec_nexigo",
                             executable = "static_transform_publisher",
                             arguments = tf_nexigo_logitec_arg)





    # tf_logitec_map = logitec_cam["tf"]["map"]
    # tf_logitec_to_nexigo = logitec_cam["tf"]["logitec"]
    # tf_logitec_map_arg = list(map(str, tf_logitec_map)) + ["logitec_cam", "map"]
    # tf_logitec_nexigo_arg = list(map(str, tf_logitec_to_nexigo)) + ["logitec_cam", "nexigo_cam"]

    # logitec_to_map = Node(package = "tf2_ros",
    #                       name="tf_logitec_map",
    #                       executable = "static_transform_publisher",
    #                       arguments = tf_logitec_map_arg)
    
    # logitec_to_nexigo = Node(package = "tf2_ros",
    #                       name="tf_logitec_nexigo",
    #                       executable = "static_transform_publisher",
    #                       arguments = tf_logitec_nexigo_arg)


    ld.add_action(nexigo_container)
    ld.add_action(logitec_container)

    # static transform
    ld.add_action(nexigo_to_logitec)
    ld.add_action(nexigo_to_map)



    return ld
