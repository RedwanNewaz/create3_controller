

add_executable(simple_controller simple_controller_node.cpp ${STATE_ESTIMATOR_SRC_FILES}
        ../Core/ControllerManager.cpp ../Core/ControllerManager.h
        SimpleController.cpp SimpleController.h ../Core/DockHelper.h)

ament_target_dependencies(
        simple_controller
        geometry_msgs
        rclcpp
        rclcpp_action
        tf2
        tf2_ros
        sensor_msgs

)

target_link_libraries(simple_controller
        ${rclcpp_LIBRARIES}
        ${rclcpp_action_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${irobot_create_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        yaml-cpp
        Eigen3::Eigen
        )

## Install
target_include_directories(simple_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(simple_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS simple_controller
        DESTINATION lib/${PROJECT_NAME})
