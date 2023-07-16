option(SIMPLE_CONTROLLER "Enable Simple PID controller Build Option" ON)
option(DWA_CONTROLLER "Enable DWA controller Build Option" OFF)
option(GAZEBO_CONTROLLER "Enable Gazebo controller Build Option" OFF)
option(WAYPOINT_CONTROLLER "Enable waypoint controller Build Option" ON)
option(MULTIROBOT "Enable multirobot Build Option" OFF)


#if(SIMPLE_CONTROLLER)
#    add_library(SimpleController
#            ${PROJECT_SOURCE_DIR}/controller/manager.h
#            ${PROJECT_SOURCE_DIR}/controller/manager.cpp
#            ${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.h
#            ${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.cpp
#            )
#    # build simple controller
#    add_executable(simple_controller ${PROJECT_SOURCE_DIR}/src/simple_controller_node.cpp )
#    target_link_libraries(SimpleController create3_model)
#    target_link_libraries(simple_controller SimpleController)
#
#    target_include_directories(simple_controller PUBLIC
#            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
#            $<INSTALL_INTERFACE:include>)
#    target_compile_features(simple_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#    install(TARGETS simple_controller DESTINATION lib/${PROJECT_NAME})
#endif ()


add_library(SimpleController
        ${PROJECT_SOURCE_DIR}/controller/manager.h
        ${PROJECT_SOURCE_DIR}/controller/manager.cpp
        ${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.h
        ${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.cpp
        )
# build simple controller
add_executable(simple_controller ${PROJECT_SOURCE_DIR}/src/simple_controller_node.cpp )
target_link_libraries(SimpleController StateEstimator)
target_link_libraries(simple_controller SimpleController)

target_include_directories(simple_controller PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>)
target_compile_features(simple_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS simple_controller DESTINATION lib/${PROJECT_NAME})





# build dwa controller
#find_package(yaml-cpp REQUIRED)
if (DWA_CONTROLLER)
    include(cmake/CPM.cmake)
    CPMAddPackage("gh:jbeder/yaml-cpp#yaml-cpp-0.6.3@0.6.3")
    include_directories(${yaml-cpp_INCLUDE_DIRS})

    add_library(DWAController
            ${PROJECT_SOURCE_DIR}/controller/manager.h
            ${PROJECT_SOURCE_DIR}/controller/manager.cpp
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.h
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.cpp
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.h
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.cpp
            ${PROJECT_SOURCE_DIR}/include/utilities/param_manager2.h
            )

    add_executable(dwa_controller src/dwa_controller_node.cpp)
    target_link_libraries(dwa_controller DWAController yaml-cpp)
    target_include_directories(dwa_controller PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>)
    target_compile_features(dwa_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS dwa_controller DESTINATION lib/${PROJECT_NAME})
endif ()

if (GAZEBO_CONTROLLER)
    add_library(GazeboController
            ${PROJECT_SOURCE_DIR}/controller/manager.h
            ${PROJECT_SOURCE_DIR}/controller/manager.cpp
            ${PROJECT_SOURCE_DIR}/model/GazeboStateEstimator.h
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.h
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner.cpp
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.h
            ${PROJECT_SOURCE_DIR}/controller/DWA/dwa_planner_ros.cpp
            ${PROJECT_SOURCE_DIR}/include/utilities/param_manager2.h
            ${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.h
            ${PROJECT_SOURCE_DIR}/controller/PID/UnicycleController.cpp
            )

    add_executable(gazebo_controller src/gazebo_sim_node.cpp)
    target_link_libraries(gazebo_controller GazeboController yaml-cpp)
    target_include_directories(gazebo_controller PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>)
    target_compile_features(gazebo_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS gazebo_controller DESTINATION lib/${PROJECT_NAME})
endif ()


if (WAYPOINT_CONTROLLER)
    find_package(action_waypoints_interfaces REQUIRED)
    add_library(waypoint_controller SHARED
            ${PROJECT_SOURCE_DIR}/controller/Waypoints/WaypointController.cpp
            ${PROJECT_SOURCE_DIR}/controller/Waypoints/WaypointController.h
            ${PROJECT_SOURCE_DIR}/controller/Waypoints/rapidcsv.h
            )
    target_include_directories(waypoint_controller PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    target_compile_definitions(waypoint_controller
            PRIVATE "WAYPOINT_CONTROLLER_CPP_BUILDING_DLL")
    ament_target_dependencies(waypoint_controller
            "action_waypoints_interfaces"
            "rclcpp"
            "geometry_msgs"
            "nav_msgs"
            "rclcpp_action"
            "rclcpp_components")
    rclcpp_components_register_node(waypoint_controller PLUGIN "navigation::WaypointController" EXECUTABLE waypoint_action_server)

    install(TARGETS
            waypoint_controller
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )
    ament_export_dependencies(rosidl_default_runtime)
endif ()


# multi-robot control

if(MULTIROBOT)
    find_package(HRVO REQUIRED)
    include_directories(/usr/local/include/HRVO)
    add_executable(multi_robot_gazebo_sim ${PROJECT_SOURCE_DIR}/src/multirobot_gazebo_sim_node.cpp)
    target_link_libraries(multi_robot_gazebo_sim create3_model create3_view yaml-cpp -lHRVO)
    target_include_directories(multi_robot_gazebo_sim PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>)
    target_compile_features(multi_robot_gazebo_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS gazebo_controller DESTINATION lib/${PROJECT_NAME})
endif()

