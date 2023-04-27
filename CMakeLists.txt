cmake_minimum_required(VERSION 3.8)
project(create3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in

find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



link_directories(/usr/local/lib)

# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include
        ${rclcpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${irobot_create_msgs_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
		${Eigen_INCLUDE_DIRS}
		/usr/include/eigen3/
		${rcl_interfaces_INCLUDE_DIRS}
		${tf2_INCLUDE_DIRS}
		${tf2_ros_INCLUDE_DIRS}
        )


set(STATE_ESTIMATOR_SRC_FILES
		${PROJECT_SOURCE_DIR}/state_estimator/EKF.cpp
		${PROJECT_SOURCE_DIR}/state_estimator/EKF.h
		${PROJECT_SOURCE_DIR}/state_estimator/FilterBase.h
		${PROJECT_SOURCE_DIR}/state_estimator/StateViz.cpp
		${PROJECT_SOURCE_DIR}/state_estimator/StateViz.h
		${PROJECT_SOURCE_DIR}/state_estimator/ComplementaryFilter.h
		${PROJECT_SOURCE_DIR}/state_estimator/StateEstimatorBase.h
		${PROJECT_SOURCE_DIR}/state_estimator/JointStateEstimator.cpp
		${PROJECT_SOURCE_DIR}/state_estimator/JointStateEstimator.h
		${PROJECT_SOURCE_DIR}/state_estimator/LoggerCSV.h
		)


add_subdirectory(state_estimator)
add_subdirectory(simple_controller)
add_subdirectory(dwa_controller)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()


# install

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME})

