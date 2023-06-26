//
// Created by redwan on 6/25/23.
//

#include "DynamicMap.h"
using namespace map_server;
DynamicMap::DynamicMap(const rclcpp::NodeOptions &options): Node("dynamic_map_server", options) {
    pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    this->declare_parameter("mapResolution", 0.0052);
    this->declare_parameter("mapOffsetX", -0.45);
    this->declare_parameter("mapOffsetY", 0.1);
    m_resolution = this->get_parameter("mapResolution").get_parameter_value().get<double>();
    m_offsetX = this->get_parameter("mapOffsetX").get_parameter_value().get<double>();
    m_offsetY = this->get_parameter("mapOffsetY").get_parameter_value().get<double>();

    this->action_server_ = rclcpp_action::create_server<Dynmap>(
            this,
            "dynmap",
            std::bind(&DynamicMap::handle_goal, this, _1, _2),
            std::bind(&DynamicMap::handle_cancel, this, _1),
            std::bind(&DynamicMap::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Dynamic Map Server is ready");
}

rclcpp_action::GoalResponse
DynamicMap::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Dynmap::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with map %s", goal->map_path.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DynamicMap::handle_cancel(const std::shared_ptr<GoalHandleDynmap> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DynamicMap::handle_accepted(const std::shared_ptr<GoalHandleDynmap> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DynamicMap::execute, this, _1), goal_handle}.detach();
}

void DynamicMap::execute(const std::shared_ptr<GoalHandleDynmap> goal_handle) {

    // Provide the path to the image file
    const auto goal = goal_handle->get_goal();
    std::string imageFilePath = goal->map_path;

    // Load the image using OpenCV
    cv::Mat image = cv::imread(imageFilePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", imageFilePath.c_str());
        return;
    }
    // Create an OccupancyGrid message
    nav_msgs::msg::OccupancyGrid mapMsg;
    mapMsg.header.frame_id = "map";
    mapMsg.info.resolution = m_resolution; // Provide the resolution of the map in meters/pixel
    mapMsg.info.width = image.cols;
    mapMsg.info.height = image.rows;

    double MID_X = mapMsg.info.width * mapMsg.info.resolution  / 2.0;
    double MID_Y = mapMsg.info.height * mapMsg.info.resolution / 2.0;
    RCLCPP_INFO(this->get_logger(), "Mid (X, Y) = (%lf, %lf)", MID_X, MID_Y);
    mapMsg.info.origin.position.x = -MID_X + m_offsetX;
    mapMsg.info.origin.position.y = -MID_Y + m_offsetY;
    mapMsg.info.origin.position.z = 0.0;
    mapMsg.info.origin.orientation.w = 1.0;

    // Populate the OccupancyGrid data from the image
    mapMsg.data.reserve(image.cols * image.rows);
    for (int row = 0; row < image.rows; ++row) {
        for (int col = 0; col < image.cols; ++col) {
            uint8_t pixelValue = image.at<uint8_t>(row, col);
            int8_t occupancyValue = (pixelValue < 128) ? 100 : 0;
            mapMsg.data.push_back(occupancyValue);
        }
    }

    // publish message
    mapMsg.header.stamp = this->now();
    pub_map_->publish(mapMsg);

    auto result = std::make_shared<Dynmap::Result>();
    if (rclcpp::ok()) {
        result->update = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

}


RCLCPP_COMPONENTS_REGISTER_NODE(map_server::DynamicMap)