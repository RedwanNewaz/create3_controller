//
// Created by redwan on 6/24/23.
//

#include "WaypointController.h"

namespace navigation
{
    class WaypointController : public rclcpp::Node
    {
    public:
        using Waypoints = action_waypoints_interfaces::action::Waypoints;
        using GoalHandleWaypoints = rclcpp_action::ServerGoalHandle<Waypoints>;

        ACTION_TUTORIALS_CPP_PUBLIC
        explicit WaypointController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
                : Node("waypoint_action_server", options)
        {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<Waypoints>(
                    this,
                    "waypoints",
                    std::bind(&WaypointController::handle_goal, this, _1, _2),
                    std::bind(&WaypointController::handle_cancel, this, _1),
                    std::bind(&WaypointController::handle_accepted, this, _1));
            pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",1);
            RCLCPP_INFO(this->get_logger(), "Waypoint Controller is ready");
        }

    private:
        struct point {
            double x;
            double y;
        };
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;


        rclcpp_action::Server<Waypoints>::SharedPtr action_server_;

        rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Waypoints::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->csv_path.c_str());
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleWaypoints> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleWaypoints> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&WaypointController::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleWaypoints> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal");


            const auto goal = goal_handle->get_goal();
//            const std::string path = "/home/redwan/colcon_ws/src/create3_controller/test/wp_test1.csv";
            const std::string path = goal->csv_path;

            if(!std::filesystem::exists(path))
            {
                RCLCPP_INFO(this->get_logger(), "csv file not found!");
                return;
            }

            rapidcsv::Document doc(path, rapidcsv::LabelParams(-1, -1));
            std::vector<float> xValues = doc.GetColumn<float>(0);
            std::vector<float> yValues = doc.GetColumn<float>(1);
            int N = xValues.size();

            std::vector<point> path_points(N);
            for (int i = 0; i < N; ++i) {
                path_points[i] = point{xValues[i], yValues[i]};
            }
            const double resolution = 0.345; //m
            const double updateFq = 0.75; //Hz
            rclcpp::Rate loop_rate(updateFq);
            auto final_path = interpolateWaypoints(path_points, resolution);
            auto feedback = std::make_shared<Waypoints::Feedback>();
            auto & sequence = feedback->time_sequence;


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
                sequence.push_back(i);
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish feedback %d", i);

                loop_rate.sleep();
            }

            // Check if goal is done
            if (rclcpp::ok()) {
                result->tracking_sequence = sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        // interpolation
        std::vector<point> interpolateWaypoints(const std::vector<point>& waypoints, double resolution) {
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
        void pubMsg(double x, double y)
        {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = this->get_clock()->now();
            msg.pose.position.x = x;
            msg.pose.position.y = y;
            pub_goal_pose_->publish(msg);
        }
    };  // class WaypointController

}  // namespace navigation

RCLCPP_COMPONENTS_REGISTER_NODE(navigation::WaypointController)