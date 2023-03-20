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


include(cmake/CPM.cmake)

CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
#add_subdirectory(third_party/BehaviorTree.CPP)
#CPMAddPackage("gh:BehaviorTree/BehaviorTree.CPP#master")

link_directories(/usr/local/lib)

# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include
        ${rclcpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${irobot_create_msgs_INCLUDE_DIRS}
        ${visualization_msgs_INCLUDE_DIRS}
#        ${CMAKE_BINARY_DIR}/_deps/behaviortree.cpp-src/include
	${rcl_interfaces_INCLUDE_DIRS}
	${tf2_INCLUDE_DIRS}
	${tf2_ros_INCLUDE_DIRS}


        )

add_library(COM include/COM.h src/COM.cpp include/utilities/param_manager2.h)
set_source_files_properties(COM.cpp PROPERTIES COMPILE_FLAGS "-include include/COM.h")




add_executable(create3_controller_node src/create3_controller_node.cpp
        src/dwa_planner.cpp
        include/create3_controller/dwa_planner.h
        include/create3_controller/state_estimator.h
        include/utilities/param_manager2.h
        include/utilities/viz_objects.h
        src/dwa_planner_ros.cpp include/create3_controller/dwa_planner_ros.h)

ament_target_dependencies(
	create3_controller_node
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
)

target_link_libraries(create3_controller_node
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${irobot_create_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        yaml-cpp
        COM
        )
add_subdirectory(state_estimator)
add_subdirectory(create3_controller_gui)
target_include_directories(create3_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(create3_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS create3_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME})

#install(
#        DIRECTORY lib/
#        DESTINATION lib
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



