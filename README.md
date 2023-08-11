# MRCP-UGVs: Multi-robot Control and Planning for Unmanned Ground Vehicles with AiRLab's Software Toolkit 

AiRLab presents the Multi-robot Control and Planning (MRCP) toolkit that enables control and planning for multiple unmanned ground vehicles (UGVs) using the ROS2 platform. This toolkit provides an integrated solution for controlling and monitoring multiple UGVs through manual operation, autonomous navigation, sensor-based localization, and real-time visualization. This system utilizes multiple iRobot Create 3 robots to demonstrate the  modular and robust multi-robot capabilities.

### Key Features
The toolkit provides a graphical user interface that allows a user to easily control multiple robots simultaneously. It also features a multi-camera AprilTag localization system that can accurately track the robots' locations in the environment. AiRLab employs sensor fusion algorithms to integrate data from the various sensors to determine the precise pose of each robot. 

In summary: 

- Gphical user interface for easy control of multiple robots
- Multi-camera AprilTag localization system for tracking robot locations
- Sensor fusion algorithms for precise pose estimation of each robot
- Individual joystick control for manual operation of each robot
- Low-level position control for waypoint navigation


### Waypoint Navigation

In addition to autonomous control, the toolkit enables manual control of the robots using joystick controllers. Each robot can be controlled independently through a single joystick. The software also provides low-level position control to navigate robots to specified waypoints. Two options are available for waypoint navigation: a PID controller for point-to-point movements and a DWA controller for collision-free navigation. Furthermore, a safety controller utilizing infrared sensors is implemented to prevent the robots from colliding with walls.

In summary: 

- PID controller for point-to-point movements
- DWA controller for collision-free navigation
- Infrared sensor based safety controller to avoid collisions
### Path Following
The waypoint controller can handle applications that require repeatedly visiting a set of locations. It accepts a csv file via the graphical user interface and executes the path using the PID controller. If an obstacle is encountered, the waypoint controller will stop and wait for the human operator to use the joystick controller to overcome the situation. This default behavior is designed to ensure safety in the event of an obstacle.

In summary: 

- Waypoint controller to designate path waypoints for robots to follow
### Visualization

Visualizing the robot's internal computation is essential for development and debugging. To this end, AiRLab uses a variety of visualization techniques to help developers understand the robot's state and behavior. For state visualization, the create3 body is used to demonstrate the real-time state of the robot. This includes the robot's position, orientation, and velocity. 

Point-to-point navigation is visualized with a yellow sphere. Users can send goals to the robot using the GUI, and the sphere will move to the desired goal. The DWA controller shows short-term paths and obstacle locations. This helps developers understand how the robot is planning its movements and how it is avoiding obstacles. The map server helps developers understand the geometric target area. This includes the size and shape of the area, as well as the obstacles that are present.

These visualization techniques are essential for developing and debugging robot software. They allow developers to see what the robot is doing and to understand how it is working. This information is critical for ensuring that the robot is safe and reliable.

In summary :

- RVIZ visualize robots' states and planning behaviors
- Map server to visualize world map with current robot locations and paths





## TODO List 
1. ~~create sensor fusion-based state estimator~~ 
   1. ~~generate raw traj from odom~~ 
   2. ~~generate raw traj from apriltag~~
   3. ~~generate raw traj from cmd_vel~~ 
   4. ~~analyze trajectory from multiple sensors using either~~ 
      5. EKF 
      6. Lowpass Filter 
   
2. ~~create gui interface using qt creator~~
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
3. ~~modify dwa based create3_controller~~
   1. integrate safety factor from ir_intensities
   2. use map for efficient navigation
   3. add dynamic collision avoidance functionality 
   4. multiple robot experiments 