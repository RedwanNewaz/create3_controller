//
// Created by roboticslab on 8/9/23.
//

#ifndef CREATE3_CONTROLLER_JOYSTICK_H
#define CREATE3_CONTROLLER_JOYSTICK_H
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define DEBOUNCE_TIME 0.2


namespace controller{
    class joystick: public rclcpp::Node
    {
    public:
        joystick(const std::vector<std::string>& namespaces): rclcpp::Node("joystick"), robotIndex_(0)
        {
            prev_ = this->now();

            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, [&](sensor_msgs::msg::Joy::SharedPtr msg)
            {
                decode_joy_msg(msg);
            });


            std::cout << "Received vector of strings: ";
            for (const std::string &str : namespaces) {
                std::cout << str << ", ";
                auto pub = this->create_publisher<geometry_msgs::msg::Twist>("/" + str + "/cmd_vel", 10);
                cmd_pubs_.emplace_back(pub);
                ns_.emplace_back(str);
            }
            std::cout << std::endl;

            cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [&](geometry_msgs::msg::Twist::SharedPtr msg)
            {
                cmd_pubs_[robotIndex_]->publish(*msg);
            });


        }

    protected:
        void decode_joy_msg(sensor_msgs::msg::Joy::SharedPtr msg)
        {
            rclcpp::Time curr = get_clock()->now();

            auto elapsed = (curr - prev_).seconds();
            // Set debounce time 150 ms
            if(elapsed < DEBOUNCE_TIME)
                return;



            auto button =  std::find(msg->buttons.begin(), msg->buttons.end(), 1);
            if(button != msg->buttons.end())
            {
                int index = std::distance(msg->buttons.begin(), button);
                //RCLCPP_INFO(get_logger(), "index = (%d)", index);

                switch (index) {
                    case 0: RCLCPP_INFO(get_logger(), "button = (X)"); break;
                    case 1: RCLCPP_INFO(get_logger(), "button = (A)"); break;
                    case 2: RCLCPP_INFO(get_logger(), "button = (B)"); break;
                    case 3: RCLCPP_INFO(get_logger(), "button = (Y)"); break;
                    case 4: robotIndex_ = (robotIndex_ + 1) % ns_.size(); RCLCPP_INFO(get_logger(), "robot = (%s)", ns_[robotIndex_].c_str()); break;
                }
            }

            prev_ = get_clock()->now();

        }


    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;
        geometry_msgs::msg::Twist::SharedPtr cmd_;
        int robotIndex_;
        rclcpp::Time prev_;
        std::vector<std::string> ns_;
    };
}
#endif //CREATE3_CONTROLLER_JOYSTICK_H
