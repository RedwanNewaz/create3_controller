//
// Created by redwan on 6/24/23.
//

#ifndef CREATE3_CONTROLLER_WAYPOINTCONTROLLER_H
#define CREATE3_CONTROLLER_WAYPOINTCONTROLLER_H
#include <functional>
#include <memory>
#include <thread>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_waypoints_interfaces/action/waypoints.hpp"
#include "rapidcsv.h"
#include <geometry_msgs/msg/pose_stamped.hpp>

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
#define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
#define ACTION_TUTORIALS_CPP_IMPORT
#if __GNUC__ >= 4
#define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
#define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
#endif
#define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif


#endif //CREATE3_CONTROLLER_WAYPOINTCONTROLLER_H
