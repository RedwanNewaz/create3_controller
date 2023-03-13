find_package(Eigen3 REQUIRED)
add_executable(create3_state_estimator state_estimator_node.cpp EKF.cpp EKF.h EKF.h FilterBase.h FilterBase.h StateViz.cpp StateViz.h ComplementaryFilter.h ComplementaryFilter.h StateEstimatorBase.h StateEstimatorBase.h JointStateEstimator.cpp JointStateEstimator.h)
include_directories(${Eigen_INCLUDE_DIRS})
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
        COM
        Eigen3::Eigen

        )


install(DIRECTORY ../launch
        DESTINATION share/${PROJECT_NAME})