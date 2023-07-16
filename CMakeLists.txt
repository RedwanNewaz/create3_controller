cmake_minimum_required(VERSION 3.8)
project(create3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options( -Wall -Wextra -Wpedantic)
#	add_link_options("-Wl,-z,relro,-z,now,-z,defs")
endif()



set(CMAKE_CXX_STANDARD 17)
cmake_policy(SET CMP0009 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Threads REQUIRED)
# uncomment the following section in order to fill in

find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
		${apriltag_msgs_INCLUDE_DIRS}
        )


add_subdirectory(model)
add_subdirectory(view)
add_subdirectory(controller)

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
	set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  	find_package(ament_lint_auto REQUIRED)
  	ament_lint_auto_find_test_dependencies()
endif()
ament_package()


# install
#cmake_policy(SET CMP0009 NEW)








