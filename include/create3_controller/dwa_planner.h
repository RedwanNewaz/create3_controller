//
// Created by Abdullah Al Redwan Newaz on 8/27/22.
//

#ifndef DWA_DWA_PLANNER_H
#define DWA_DWA_PLANNER_H
#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include <functional>

//#define PI 3.141592653

using Traj = std::vector<std::array<double, 5>>;
using Obstacle = std::vector<std::array<double, 2>>;
using State = std::array<double, 5>;
using Window = std::array<double, 4>;
using Point = std::array<double, 2>;
using Control = std::array<double, 2>;

namespace DynamicWindow{

    class Config{
    public:
        double max_speed = 0.345;
        double min_speed = -0.345;
        double max_yawrate = 60.0 * M_PI / 180.0;
        double max_accel = 0.4;
        double robot_radius = 0.345 / 2;
        double max_dyawrate = 60.0 * M_PI / 180.0;

        double v_reso = 0.01;
        double yawrate_reso = 0.2 * M_PI / 180.0;

        double dt = 0.03;
        double predict_time = 3.0;
        double to_goal_cost_gain = 1.0;
        double speed_cost_gain = 0.10;
        double to_obstacle_cost_gain = 1.0;

        double goal_radius = 1.0;


        void update_param(std::function<double(const std::string&)>&f)
        {
            max_speed = f("max_speed");
            min_speed = f("min_speed");
            max_yawrate = f("max_yawrate");
            max_accel = f("max_accel");
            robot_radius = f("robot_radius");
            max_dyawrate = f("max_dyawrate");

            v_reso = f("v_reso");
            yawrate_reso = f("yawrate_reso");

            dt = f("dt");
            predict_time = f("predict_time");
            to_goal_cost_gain = f("to_goal_cost_gain");
            speed_cost_gain = f("speed_cost_gain");
            to_obstacle_cost_gain = f("to_obstacle_cost_gain");

            goal_radius = f("goal_radius");
        }
    };

    class planner {
    public:
        Traj compute_control(const State& x, Control & u, const Config& config,
                             const Point& goal, const Obstacle& ob);
        State motion(State x, Control u, double dt);
        Window get_window(double predict_time);
    protected:

        Window calc_dynamic_window(const State& x, const Config& config);
        Traj calc_trajectory(State x, double v, double y, Config config);

        Traj calc_final_input(
                const State& x, Control& u,
                const Window& dw, const Config& config, const Point& goal,
                const std::vector<std::array<double, 2>>&ob);

    private:
        double calc_obstacle_cost(const Traj& traj, const Obstacle& ob, const Config& config);
        double calc_to_goal_cost(const Traj& traj, const Point& goal, const Config& config);
        Window m_dw;

    };

}


#endif //DWA_DWA_PLANNER_H
