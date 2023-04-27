#Quick Run 

start apriltag 
```bash 
ros2 launch create3_controller create3_apriltag.launch.py 
```

start joystick controller 
```bash
ros2 launch create3_controller create3_joystick.py 
```

state estimator 
```bash 
ros2 run create3_controller create3_state_estimator --ros-args -p sensor:=fusion
```

start rviz
```bash 
rviz2 -d /home/roboticslab/colcon_ws/src/create3_controller/config/create3_state.rviz
```