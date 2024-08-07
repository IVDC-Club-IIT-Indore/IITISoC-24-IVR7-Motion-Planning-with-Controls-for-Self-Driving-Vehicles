# IITISoC-24-IVR7-Motion-Planning-with-Controls-for-Self-Driving-Vehicles

## Goal:
To develop a reliable Motion Planner with Control algorithms for collision-free path planning and efficient navigation in a simulated environment.

People Involved : 

Mentors:
- [Arjun S Nair](https://github.com/arjun-593)
- [Ampady B R](https://github.com/ampady06)

Members:
<br>
- [M N Yugendran](https://github.com/user-230087)
- [Pohrselvan ss](https://github.com/pohrselvan)
- [Swarangi Kale](https://github.com/Swarangi-codes)
- [Jagrit](https://github.com/idJagrit)

# Project Overview & Solution
A three-wheeled vehicle is modeled and is visualized in gazebo simulator and RViz. The world file provided is mapped using Simeltaneous Mapping and Loacalization (SLAM) method. For Path planning RRT* algorithm were used on the map generated and PID algorithm is used for control algorithm. 

## Vehicle Modelling
Here we explains how to model a three-wheeled vehicle using URDF files and Xacro in ROS2, focusing on splitting the model into components for better organization and understanding:
### Prerequisites

- ROS2 Humble
- Gazebo classic version
**Note:** For downloading ros2 humble look into ros2 humble documents : https://docs.ros.org/en/humble/Tutorials.html
### 1.Create aworkspace 
```bash
mkdir ~/ros2_ws/src
   cd ~/ros2_ws/src
```
### 2.clone the repo
```bash
git clone https://github.com/IVDC-Club-IIT-Indore/IITISoC-24-IVR7-Motion-Planning-with-Controls-for-Self-Driving-Vehicles.git
```

### 2.Build the workspace with the colcon build
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### File discription
`robot.urdf.xacro`-main file contains other conmponents of the file such as chassis,lidar,imu
`chassis.xacro`-contains upper and lower part of the chassis
`robot_core.xacro`-contains the right and left wheel, caster wheel
`inertial.xacro`-contains the interial properties of the vechile
`lidar.xacro`-contains details of the lidar and pulgins foe the lidar
`imu.xacro`- has the details of the imu sensors and plugin 
`ros2_control_wheel.xacro`-this file for define the diff_controllers for the better control over the vechile

Launching the vehicle in the gazebo
```bash
ros2 launch july_9 launch_sim.launch.py
```
![image](https://github.com/user-attachments/assets/502b84b2-2b00-4ef5-bf01-afd9d56e1b17)


Launching the vehicle in the gazebo with world
```bash
ros2 launch july_9 new_launch_sim.launch.py
```

During SLAM the lidar range extended to infinity, due to which queue size got filled up and therefore was not able to send messages to RViz.


## Mapping the world file
The orange_igvc.world world file is mapped using SLAM method with the help of Navigation2 stack of ROS2 and turtlebot3 robot.
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```
Go to the bashrc file and set TURTLEBOT3_MODEL=waffle

Run the launch file to open the orange_igvc world file in gazebo and run the turtlebot3_teleop node to control the turtlebot3 robot. To visualize it in RViz the turtlebot3_cartographer is used and the map is saved using nav2_map_server

```bash
ros2 run nav2_map_server map_saver_cli -f maps/maps2
```

```bash
cd maps
ls
```

On opening the .pgm file the map we get

![world_map](https://github.com/user-attachments/assets/9f6fa071-4bf5-4b78-aa47-e052c8d36fa7)

## Path Planning
Path Planning gives the vehicle's trajectory from its starting state to some goal state in the map. There are most popularly two approaches for path planning - Search based methods & Sampling based methods. In Search based algorithms, cost to access each node from start node in graph is calculated, to get the most optimal or shortest path to reach the goal node. Common examples being Dijkstra's and A* algorithms. As the size of map and dimensions increases they become computational expensive but still give us the shortest possible path.


Sampling based algorithms doesn't require fully exploring the configurational space and it also requires lesser computational power, though it doesn't give the most optimal path as output. RRT* Algorithm( modification of RRT ), which gives the nearest to most optimal path for the robot, is an example of sampling based algorithm. 


The RRT* algorithm has been used for path planning. The code implementation takes the binary occupancy grid map as .npy file for detecting the obstacles and the start point, goal point and waypoints are given as input co-ordinates. The final path is visualized using the matplotlib library.

Visualisation of waypoints is as follows:

![](https://github.com/user-attachments/assets/f41e8b76-5f71-4691-84c8-5bcb451fe8fe)

Visualisation of path planned on the binary occupancy map is as follows:

![](https://github.com/user-attachments/assets/a00cb808-e082-4d23-88b0-e95432832c42)

**Comparison of Path Planning Algorithms**
https://docs.google.com/spreadsheets/d/17M89Dh4DRQxYJfTK4u48PiHGHFnKuStqf_kaLztgRA0/edit?gid=0#gid=0 

## Control Algorithm
### Path Following and Control Algorithm

* The path following (Controller) algorithm controls the bot to follow the waypoints generated by path planning algorithm. It consists of two types of controls, longitudinal and lateral. Longitudinal control regulates the cruise velocity of the vehicle while the lateral control steers the bot to follow the path.
* Closed-loop controls like black box controls, geometric controls and optimal controls are widely used for these purposes. This project uses PID as a black box control method for both lateral and longitudinal controls.
* The errors, namely Cross Track Error (CTE) and Orientation Error that can occur during autonomous driving can be minimized and subsequently eliminated using the control algorithms.

**Process flow**
* The algorithm inputs:
   1. Current position and orientation of the bot from odometer. (/diff_cont/odom topic)
   2. Angular velocity of the bot from IMU sensor (/imu_plugin/out topic)
  This is done by creating a node in the algorithm that subscribes to the respective topics and recieves the data 
* Then, the CTE and orientation errors are calculated using this data.
* These are minimized by controlling the linear and angular velocity of the bot using two seperate PID loops.
* These values are published to the cmd_vel topic using a publisher created in the same node.

  ## Sources

  https://youtu.be/O8RENvOxbTY?si=LwuDwQEdjK0KXRDY</br>
  https://youtu.be/UR0hOmjaHp0?si=MUCW_nwGLe8yaLhb</br>
  https://youtu.be/idQb2pB-h2Q?si=YXm61nxTsX0w9an5</br>
  https://github.com/Tinker-Twins/Self_Driving_Car_Trajectory_Tracking/blob/main/Controller.py</br>
  https://github.com/AtsushiSakai/PythonRobotics/tree/master?tab=readme-ov-file#rapidly-exploring-random-trees-rrt</br>

  We will be implementing the path planner and the controller on our bot in RViz after this.


