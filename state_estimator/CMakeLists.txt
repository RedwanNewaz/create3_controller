add_executable(create3_state_estimator state_estimator_node.cpp ${STATE_ESTIMATOR_SRC_FILES})

ament_target_dependencies(
        create3_state_estimator
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
)

target_link_libraries(create3_state_estimator
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${irobot_create_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        yaml-cpp
        Eigen3::Eigen
        )


#install(DIRECTORY ../launch
#        DESTINATION share/${PROJECT_NAME})


target_include_directories(create3_state_estimator PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/state_estimator>
        $<INSTALL_INTERFACE:include>)
target_compile_features(create3_state_estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS create3_state_estimator
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ../launch
        DESTINATION share/${PROJECT_NAME})

