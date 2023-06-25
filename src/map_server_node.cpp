//
// Created by redwan on 6/25/23.
//
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("map_publisher_node");

    // Provide the path to the image file
    std::string imageFilePath = "/home/redwan/colcon_ws/src/create3_controller/config/bak/vector_field1.png";

    // Load the image using OpenCV
    cv::Mat image = cv::imread(imageFilePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load image: %s", imageFilePath.c_str());
        return 1;
    }

    // Create an OccupancyGrid message
    nav_msgs::msg::OccupancyGrid mapMsg;
    mapMsg.header.frame_id = "map";
    mapMsg.info.resolution = 0.003; // Provide the resolution of the map in meters/pixel
    mapMsg.info.width = image.cols;
    mapMsg.info.height = image.rows;

    double MID_X = mapMsg.info.width * mapMsg.info.resolution  / 2.0;
    double MID_Y = mapMsg.info.height * mapMsg.info.resolution / 2.0;
    RCLCPP_INFO(node->get_logger(), "Mid (X, Y) = (%lf, %lf)", MID_X, MID_Y);
    mapMsg.info.origin.position.x = -MID_X;
    mapMsg.info.origin.position.y = -MID_Y;
    mapMsg.info.origin.position.z = 0.0;
    mapMsg.info.origin.orientation.w = 1.0;

    // Populate the OccupancyGrid data from the image
    mapMsg.data.reserve(image.cols * image.rows);
    for (int row = 0; row < image.rows; ++row) {
        for (int col = 0; col < image.cols; ++col) {
            uint8_t pixelValue = image.at<uint8_t>(row, col);
            int8_t occupancyValue = (pixelValue == 0) ? 0 : 100;
            mapMsg.data.push_back(occupancyValue);
        }
    }

    // Create a publisher to publish the OccupancyGrid message
    auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Publish the map message periodically
    rclcpp::WallRate loopRate(1); // Adjust the publishing rate as needed
    while (rclcpp::ok()) {
        mapMsg.header.stamp = node->now();
        publisher->publish(mapMsg);
        rclcpp::spin_some(node);
        loopRate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
