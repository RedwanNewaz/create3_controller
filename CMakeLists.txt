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


include_directories(
		include
		include/utilities
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




set(create3_model_SRC_FILES
		${PROJECT_SOURCE_DIR}/model/filter/EKF.cpp
		${PROJECT_SOURCE_DIR}/model/filter/EKF.h
		${PROJECT_SOURCE_DIR}/model/filter/FilterBase.h
		${PROJECT_SOURCE_DIR}/model/filter/ComplementaryFilter.h
		${PROJECT_SOURCE_DIR}/model/StateEstimatorBase.h
		${PROJECT_SOURCE_DIR}/model/JointStateEstimator.cpp
		${PROJECT_SOURCE_DIR}/model/JointStateEstimator.h
		${PROJECT_SOURCE_DIR}/include/utilities/LoggerCSV.h
		${PROJECT_SOURCE_DIR}/model/GazeboStateEstimator.h)

add_library(create3_model ${create3_model_SRC_FILES})
target_link_libraries(create3_model
		${rclcpp_LIBRARIES}
		${geometry_msgs_LIBRARIES}
		${irobot_create_msgs_LIBRARIES}
		${nav_msgs_LIBRARIES}
		${tf2_ros_LIBRARIES}
		Eigen3::Eigen
		)

set(create2_view_SRC_FILES
		${PROJECT_SOURCE_DIR}/view/StateViz.cpp
		${PROJECT_SOURCE_DIR}/view/StateViz.h
		${PROJECT_SOURCE_DIR}/view/ControllerViz.h
		${PROJECT_SOURCE_DIR}/view/ControllerViz.cpp
		)

#[Note]:		yaml-cpp library is not used in here
add_library(create3_view ${create2_view_SRC_FILES})
target_link_libraries(create3_view ${visualization_msgs_LIBRARIES})
add_executable(create3_state_estimator src/state_estimator_node.cpp)
target_link_libraries(create3_state_estimator create3_model create3_view )


# build simple controller
add_executable(simple_controller src/simple_controller_node.cpp
		${PROJECT_SOURCE_DIR}/controller/manager.h
		${PROJECT_SOURCE_DIR}/controller/manager.cpp
		${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.h
		${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.cpp
		)
target_link_libraries(simple_controller create3_model create3_view)


# build dwa controller
#find_package(yaml-cpp REQUIRED)
include(cmake/CPM.cmake)
CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
include_directories(${yaml-cpp_INCLUDE_DIRS})
add_executable(dwa_controller src/dwa_controller_node.cpp
		${PROJECT_SOURCE_DIR}/controller/manager.h
		${PROJECT_SOURCE_DIR}/controller/manager.cpp
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.h
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.cpp
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.h
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.cpp
		${PROJECT_SOURCE_DIR}/include/utilities/param_manager2.h
		)
target_link_libraries(dwa_controller create3_model create3_view yaml-cpp)


add_executable(gazebo_controller src/gazebo_sim_node.cpp
		${PROJECT_SOURCE_DIR}/controller/manager.h
		${PROJECT_SOURCE_DIR}/controller/manager.cpp
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.h
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.cpp
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.h
		${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.cpp
		${PROJECT_SOURCE_DIR}/include/utilities/param_manager2.h
		${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.h
		${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.cpp
		)
target_link_libraries(gazebo_controller create3_model create3_view yaml-cpp)

find_package(HRVO REQUIRED)
include_directories(/usr/local/include/HRVO)
add_executable(multi_robot_gazebo_sim src/multirobot_gazebo_sim_node.cpp)
target_link_libraries(multi_robot_gazebo_sim create3_model create3_view yaml-cpp -lHRVO)

add_subdirectory(view/gui)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()


# install
cmake_policy(SET CMP0009 NEW)

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME})




target_include_directories(create3_state_estimator PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>)
target_compile_features(create3_state_estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS create3_state_estimator DESTINATION lib/${PROJECT_NAME})

target_include_directories(dwa_controller PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>)
target_compile_features(dwa_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS dwa_controller DESTINATION lib/${PROJECT_NAME})

target_include_directories(simple_controller PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>)
target_compile_features(simple_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS simple_controller DESTINATION lib/${PROJECT_NAME})


target_include_directories(gazebo_controller PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>)
target_compile_features(gazebo_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS gazebo_controller DESTINATION lib/${PROJECT_NAME})

target_include_directories(multi_robot_gazebo_sim PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>)
target_compile_features(multi_robot_gazebo_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS gazebo_controller DESTINATION lib/${PROJECT_NAME})


