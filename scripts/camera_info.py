#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import yaml

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.timer_ = self.create_timer(0.001, self.publish_camera_info)
        with open('../config/head_camera_nexigo_1920.yaml') as file:
            self.yaml_data = yaml.safe_load(file)
        self.subscription_ = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription_  # Enable message reception
        self.header = None

    def image_callback(self, msg):
        self.header = msg.header
    def publish_camera_info(self):
        camera_info_msg = CameraInfo()
        # Set the fields of the CameraInfo message
        if self.header is None:
            return
        camera_info_msg.header = self.header
        # camera_info_msg.header.frame_id = 'camera'  # Set the frame ID as required
        # camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.width = self.yaml_data['image_width']
        camera_info_msg.height = self.yaml_data['image_height']
        camera_info_msg.distortion_model = self.yaml_data['distortion_model']
        camera_info_msg.d = list(map(float, self.yaml_data['distortion_coefficients']['data']))
        camera_info_msg.k = list(map(float, self.yaml_data['camera_matrix']['data']))
        camera_info_msg.r = list(map(float, self.yaml_data['rectification_matrix']['data']))
        camera_info_msg.p = list(map(float, self.yaml_data['projection_matrix']['data']))
        # Set other relevant fields as required

        self.publisher_.publish(camera_info_msg)
        self.get_logger().info('Published CameraInfo message')
        self.header = None

def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
