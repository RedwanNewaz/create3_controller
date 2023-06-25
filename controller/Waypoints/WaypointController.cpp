//
// Created by redwan on 6/24/23.
//

#include "WaypointController.h"

namespace navigation
{
        WaypointController::WaypointController(const rclcpp::NodeOptions & options)
                : Node("waypoint_action_server", options)
        {
            using namespace std::placeholders;
            this->declare_parameter("updateFq", 0.75);
            this->declare_parameter("pathResolution", 0.345);
            this->declare_parameter("robotTopic", "odom");

            m_updateFq = this->get_parameter("updateFq").get_parameter_value().get<double>();
            m_resolution = this->get_parameter("pathResolution").get_parameter_value().get<double>();
            auto robotTopic = this->get_parameter("robotTopic").get_parameter_value().get<std::string>();

            m_odom_init = false;
            obs_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(robotTopic, 10, [&]
                (const nav_msgs::msg::Odometry::SharedPtr msg){
                //convert odom message to waypoint
                m_robot.x = msg->pose.pose.position.x;
                m_robot.y = msg->pose.pose.position.y;
                m_odom_init = true;
            });


            this->action_server_ = rclcpp_action::create_server<Waypoints>(
                    this,
                    "waypoints",
                    std::bind(&WaypointController::handle_goal, this, _1, _2),
                    std::bind(&WaypointController::handle_cancel, this, _1),
                    std::bind(&WaypointController::handle_accepted, this, _1));
            pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",1);
            RCLCPP_INFO(this->get_logger(), "Waypoint Controller is ready");
        }

        rclcpp_action::GoalResponse WaypointController::handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Waypoints::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->csv_path.c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse WaypointController::handle_cancel(
                const std::shared_ptr<GoalHandleWaypoints> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void WaypointController::handle_accepted(const std::shared_ptr<GoalHandleWaypoints> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&WaypointController::execute, this, _1), goal_handle}.detach();
        }

        void WaypointController::execute(const std::shared_ptr<GoalHandleWaypoints> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");


            const auto goal = goal_handle->get_goal();
            const std::string path = goal->csv_path;
            if(!std::filesystem::exists(path))
            {
                RCLCPP_INFO(this->get_logger(), "csv file not found!");
                return;
            }

            // read csv file values
            rapidcsv::Document doc(path, rapidcsv::LabelParams(-1, -1));
            std::vector<float> xValues = doc.GetColumn<float>(0);
            std::vector<float> yValues = doc.GetColumn<float>(1);
            int N = xValues.size();

            //convert it to a path: vector of waypoints
            std::vector<point> path_points(N);
            for (int i = 0; i < N; ++i) {
                path_points[i] = point{xValues[i], yValues[i]};
            }

            // interpolate the path based on a fixed distance
            auto final_path = interpolateWaypoints(path_points, m_resolution);

            auto feedback = std::make_shared<Waypoints::Feedback>();
            auto & sequence = feedback->time_sequence;
            rclcpp::Rate loop_rate(m_updateFq);

            // wait until odom is initialized
            while(!m_odom_init)
            {
                RCLCPP_INFO(this->get_logger(), "robot state (odom) is not initialized yet");
                loop_rate.sleep();
            }


            auto result = std::make_shared<Waypoints::Result>();
            for (std::size_t i = 0; (i < final_path.size()) && rclcpp::ok(); ++i) {
                // Check if there is a cancel request
                if (goal_handle->is_canceling()) {
                    result->tracking_sequence = sequence;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                // Update sequence
                auto curr_wp = final_path.at(i);
                pubMsg(curr_wp.x, curr_wp.y);

                double remainDist = curr_wp - m_robot;
                sequence.push_back(remainDist);
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish wp %02zu feedback %lf", i + 1, remainDist);

                // let the robot minimize the far distance
                do {
                    remainDist = curr_wp - m_robot;
                    loop_rate.sleep();
                }while(remainDist > 2 * m_resolution);

            }

            // Check if goal is done
            if (rclcpp::ok()) {
                result->tracking_sequence = sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        // interpolation
        std::vector<point> WaypointController::interpolateWaypoints(const std::vector<point>& waypoints, double resolution) {
            std::vector<point> interpolatedWaypoints;

            // Iterate through each pair of waypoints
            for (size_t i = 0; i < waypoints.size() - 1; ++i) {
                const point& start = waypoints[i];
                const point& end = waypoints[i + 1];

                // Calculate the distance between start and end waypoints
                double dx = end.x - start.x;
                double dy = end.y - start.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                // Calculate the number of interpolated points between start and end waypoints
                int numInterpolatedPoints = std::ceil(distance / resolution);

                // Calculate the step size for interpolation
                double stepSize = 1.0 / numInterpolatedPoints;

                // Interpolate and add the waypoints
                for (int j = 0; j <= numInterpolatedPoints; ++j) {
                    double t = stepSize * j;
                    double interpolatedX = start.x + t * dx;
                    double interpolatedY = start.y + t * dy;
                    interpolatedWaypoints.push_back({interpolatedX, interpolatedY});
                }
            }

            return interpolatedWaypoints;
        }

        // generate goal message
        void WaypointController::pubMsg(double x, double y)
        {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = this->get_clock()->now();
            msg.pose.position.x = x;
            msg.pose.position.y = y;
            pub_goal_pose_->publish(msg);
        }


}  // namespace navigation

RCLCPP_COMPONENTS_REGISTER_NODE(navigation::WaypointController)

//ros2 action send_goal /waypoints action_waypoints_interfaces/action/Waypoints  "{csv_path: /home/redwan/colcon_ws/src/create3_controller/test/wp_test1.csv}"