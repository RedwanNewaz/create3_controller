set(create3_view_SRC_FILES
        ${PROJECT_SOURCE_DIR}/view/StateViz.cpp
        ${PROJECT_SOURCE_DIR}/view/StateViz.h
        ${PROJECT_SOURCE_DIR}/view/ControllerViz.h
        ${PROJECT_SOURCE_DIR}/view/ControllerViz.cpp
        )

#[Note]:		yaml-cpp library is not used in here
#add_library(create3_view ${create3_view_SRC_FILES})
#target_link_libraries(create3_view ${visualization_msgs_LIBRARIES})

add_executable(create3_view_node ${PROJECT_SOURCE_DIR}/src/viewer_node.cpp ${create3_view_SRC_FILES})
target_link_libraries(create3_view_node
        ${visualization_msgs_LIBRARIES}
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        )
add_subdirectory(GUI)


target_include_directories(create3_view_node PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>)
target_compile_features(create3_view_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS create3_view_node DESTINATION lib/${PROJECT_NAME})