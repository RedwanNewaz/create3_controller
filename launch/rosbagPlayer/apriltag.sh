ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/image_raw \
    -r camera_info:=/camera/camera_info \
    --params-file /home/redwan/colcon_ws/src/create3_controller/config/apriltag_param.yaml