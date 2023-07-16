option(MAP_SERVER "Enable map server Build Option" ON)

set(create3_model_SRC_FILES
        ${PROJECT_SOURCE_DIR}/model/filter/EKF.cpp
        ${PROJECT_SOURCE_DIR}/model/filter/EKF.h
        ${PROJECT_SOURCE_DIR}/model/filter/FilterBase.h
        ${PROJECT_SOURCE_DIR}/model/filter/ComplementaryFilter.h
        ${PROJECT_SOURCE_DIR}/model/StateEstimatorBase.h
        ${PROJECT_SOURCE_DIR}/include/utilities/LoggerCSV.h
        ${PROJECT_SOURCE_DIR}/include/utilities/FusedData.h
        ${PROJECT_SOURCE_DIR}/model/JointStateEstimator.cpp
        ${PROJECT_SOURCE_DIR}/model/JointStateEstimator.h
        )

#add_library(create3_model ${create3_model_SRC_FILES})
#target_link_libraries(create3_model
#        ${rclcpp_LIBRARIES}
#        ${geometry_msgs_LIBRARIES}
#        ${irobot_create_msgs_LIBRARIES}
#        ${nav_msgs_LIBRARIES}
#        ${tf2_ros_LIBRARIES}
#        ${apriltag_msgs_LIBRARIES}
#        Eigen3::Eigen)

add_library(StateEstimator SHARED ${create3_model_SRC_FILES})
target_include_directories(StateEstimator PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(StateEstimator
        PRIVATE "StateEstimator_CPP_BUILDING_DLL")
ament_target_dependencies(StateEstimator
        rclcpp
        rclcpp_components
        geometry_msgs
        irobot_create_msgs
        nav_msgs
        tf2_ros
        apriltag_msgs
        )
ament_export_libraries(StateEstimator)
rclcpp_components_register_node(StateEstimator PLUGIN "model::JointStateEstimator" EXECUTABLE "StateEstimator_node")
ament_environment_hooks(${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH})
install(TARGETS StateEstimator
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )



#add_executable(create3_state_estimator ${PROJECT_SOURCE_DIR}/src/state_estimator_node.cpp)
#target_link_libraries(create3_state_estimator create3_model )
#target_compile_features(StateEstimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#install(TARGETS StateEstimator DESTINATION lib/${PROJECT_NAME})
#

if (MAP_SERVER)
    find_package(OpenCV REQUIRED)
    find_package(action_waypoints_interfaces REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_library(map_server SHARED
            ${PROJECT_SOURCE_DIR}/model/map/DynamicMap.cpp
            ${PROJECT_SOURCE_DIR}/model/map/DynamicMap.h
            )
    target_include_directories(map_server PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    target_compile_definitions(map_server
            PRIVATE "MAP_SERVER_CPP_BUILDING_DLL")
    ament_target_dependencies(map_server
            "action_waypoints_interfaces"
            "rclcpp"
            "OpenCV"
            "geometry_msgs"
            "nav_msgs"
            "rclcpp_action"
            "rclcpp_components")
    rclcpp_components_register_node(map_server PLUGIN "map_server::DynamicMap" EXECUTABLE dynamic_map_server)

    install(TARGETS
            map_server
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )
    ament_export_dependencies(rosidl_default_runtime )

    #	add_executable(map_server src/map_server_node.cpp)
    #	target_link_libraries(map_server ${OpenCV_LIBRARIES} create3_model)

endif ()



