include(../cmake/CPM.cmake)
CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")

set(DWA_CONTROLLER_SRC_FILES
        dwa_planner.cpp
        dwa_planner_ros.cpp
       )

add_executable(dwa_controller dwa_controller_node.cpp ${DWA_CONTROLLER_SRC_FILES} ${STATE_ESTIMATOR_SRC_FILES})

ament_target_dependencies(
        dwa_controller
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
)

target_link_libraries(dwa_controller
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${irobot_create_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        yaml-cpp
        Eigen3::Eigen
        )

## Install
target_include_directories(dwa_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(dwa_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS dwa_controller
        DESTINATION lib/${PROJECT_NAME})