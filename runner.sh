gazebo()
{
    namespace='ac31'
    ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py namespace:=$namespace use_rviz:=false use_gazebo_gui:=false spawn_dock:=false
}

addRobot()
{
    namespace='ac32'
    ros2 launch irobot_create_gazebo_bringup create3_spawn.launch.py namespace:=$namespace x:=1.0 use_rviz:=false spawn_dock:=false
}
source /root/colcon_ws/install/setup.sh
gazebo & addRobot