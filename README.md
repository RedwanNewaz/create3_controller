# create3_controller

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

## TODO List 
1. ~~create sensor fusion-based state estimator~~ 
   1. ~~generate raw traj from odom~~ 
   2. ~~generate raw traj from apriltag~~
   3. ~~generate raw traj from cmd_vel~~ 
   4. ~~analyze trajectory from multiple sensors using either~~ 
      5. EKF 
      6. Lowpass Filter 
   
2. create gui interface using qt creator
   1. simulation 
      1. gazebo 
      2. ros bag
      3. Filter options 
          1. EKF
          2. LowPass
   2. experiment 
      1. Node dependency 
         1. joystic 
         2. map 
         3. DWA
         4. state_estimator 
      2. Filter options 
         1. EKF 
         2. LowPass
   3. Node dependency
      1. bag player
      2. gazebo
      3. state_estimator
      4. DWA
   4. create a desktop icon 
3. modify dwa based create3_controller 
   1. integrate safety factor from ir_intensities
   2. use map for efficient navigation
   3. add dynamic collision avoidance functionality 
   4. multiple robot experiments 