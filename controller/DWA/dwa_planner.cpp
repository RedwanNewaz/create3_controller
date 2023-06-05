//
// Created by Abdullah Al Redwan Newaz on 8/27/22.
//

#include "dwa_planner.h"

namespace controller
{

    Traj
    DynamicWindow::planner::compute_control(const State &x, Control &u, const DynamicWindow::Config &config, const Point &goal,
                                            const Obstacle &ob) {
        // # Dynamic Window control
        m_dw = calc_dynamic_window(x, config);
        Traj traj = calc_final_input(x, u, m_dw, config, goal, ob);

        return traj;
    }

    State DynamicWindow::planner::motion(State x, Control u, double dt) {



        x[2] += u[1] * dt;
        x[2] = fmod(x[2] + M_PI, 2 * M_PI) - M_PI;

        x[0] += u[0] * std::cos(x[2]) * dt;
        x[1] += u[0] * std::sin(x[2]) * dt;
        x[3] = u[0];
        x[4] = u[1];
        return x;
    }

    Window DynamicWindow::planner::calc_dynamic_window(const State &x, const DynamicWindow::Config &config) {
        return {{
                        std::max((x[3] - config.max_accel * config.dt), config.min_speed),
                        std::min((x[3] + config.max_accel * config.dt), config.max_speed),
                        std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
                        std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)
                }};
    }

    Traj DynamicWindow::planner::calc_trajectory(State x, double v, double y, DynamicWindow::Config config) {
        Traj traj;
        traj.push_back(x);
        double time = 0.0;
        while (time <= config.predict_time){
            x = motion(x, std::array<double, 2>{{v, y}}, config.dt);
            traj.push_back(x);
            time += config.dt;
        }
        return traj;
    }

    double DynamicWindow::planner::calc_obstacle_cost(const Traj& traj, const Obstacle& ob, const DynamicWindow::Config& config) {
        // calc obstacles_ cost inf: collistion, 0:free
        int skip_n = 2;
        double minr = std::numeric_limits<double>::max();

        for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
            for (unsigned int i=0; i< ob.size(); i++){
                double ox = ob[i][0];
                double oy = ob[i][1];
                double dx = traj[ii][0] - ox;
                double dy = traj[ii][1] - oy;

                double r = std::sqrt(dx*dx + dy*dy);
                if (r <= config.robot_radius){
                    return std::numeric_limits<double>::max();
                }

                if (minr >= r){
                    minr = r;
                }
            }
        }

        return 1.0 / minr;
    }

    double DynamicWindow::planner::calc_to_goal_cost(const Traj& traj, const Point& goal, const DynamicWindow::Config& config) {
//    double goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
//    double traj_magnitude = std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
//    double dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
//    double error = dot_product / (goal_magnitude * traj_magnitude);
//    double error_angle = std::acos(error);
//    double cost =  error_angle;
        double minCost = std::numeric_limits<double>::max();
        for(const auto & state : traj) {
            auto dx = goal[0] - state[0];
            auto dy = goal[1] - state[1];
            double cost = sqrt(dx * dx + dy * dy);
            minCost = std::min(minCost, cost);
        }

        return minCost;
    }

    Traj DynamicWindow::planner::calc_final_input(const State &x, Control &u, const Window &dw,
                                                  const DynamicWindow::Config &config, const Point &goal,
                                                  const std::vector<std::array<double, 2>> &ob) {
        double min_cost = std::numeric_limits<double>::max();
        Control min_u;
        min_u[0] = min_u[1] = 0.0;
        Traj best_traj;

        // evalucate all trajectory with sampled input in dynamic window
        for (double v=dw[0]; v<=dw[1]; v+=config.v_reso){
            for (double y=dw[2]; y<=dw[3]; y+=config.yawrate_reso){

                Traj raw_traj = calc_trajectory(x, v, y, config);
                //filter points based on the goal
                Traj traj;
                for(auto& p: raw_traj)
                {
                    double dx = goal[0] - p[0];
                    double dy = goal[1] - p[1];
                    auto distance = sqrt(dx * dx + dy * dy);
                    if(distance < config.goal_radius)
                        break;
                    traj.push_back(p);
                }

                double to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(traj, goal, config);
                double speed_cost = config.speed_cost_gain * (config.max_speed - traj.back()[3]);
                double ob_cost = config.to_obstacle_cost_gain * calc_obstacle_cost(traj, ob, config);

                double final_cost =to_goal_cost + speed_cost + ob_cost;

                if (min_cost >= final_cost){
                    min_cost = final_cost;
                    min_u = Control{{v, y}};
//                int N = traj.size() / 2;
//                min_u = Control{{traj[N][3], traj[N][4]}};
                    best_traj = traj;
                }
            }
        }
//        std::cout <<"[mincost] = " << min_cost << std::endl;
        u = min_u;
        return best_traj;
    }

    Window DynamicWindow::planner::get_window(double predict_time) {
        Window window;
        window[0] =  predict_time * m_dw[0] ;
        window[1] =  predict_time * m_dw[1] ;
        window[2] =  predict_time * m_dw[0] ;
        window[3] =  predict_time * m_dw[1] ;
        return window;
    }
}