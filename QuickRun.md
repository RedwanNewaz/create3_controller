#Quick Run 

## Change configuration 
config ``config/params.yaml`` file before execution to avoid runtime errors.
Two file paths need to be changed based on your absolute folder paths. 

## Necessary commands

#### compile node
```bash 
colcon build --packages-select create3_controller --symlink-install
```

#### execute node 
```bash
ros2 launch create3_controller create3_controller_launch.yaml 
```

#### gazebo simulation 
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py x:=-0.5 y:=1.76
```

##### disable safety limits on gazebo 
```bash
ros2 param set /motion_control safety_override full
```

### state estimator ard 

```bash 
--ros-args -p sensor:=fusion
```

## create ros package 

```bash 
ros2 pkg create --build-type ament_cmake --node-name create3_path_follower create3_path_follower \
    --dependencies rclcpp tf visualization_msgs nav_msgs geometry_msgs irobot_create_msgs
```

start apriltag 
```bash 
ros2 launch create3_controller create3_apriltag.launch.py 
```

start joystick controller 
```bash
ros2 launch create3_controller create3_joystick.py 
```

## Debugging state estimator

create3 controller uses apriltga state estimator for localization.
To make sure state estimator is working as expected, we need to manually check its performance.


state estimator 
```bash 
ros2 run create3_controller create3_state_estimator --ros-args -p sensor:=fusion
```

start rviz
```bash 
rviz2 -d /home/roboticslab/colcon_ws/src/create3_controller/config/create3_state.rviz
```



