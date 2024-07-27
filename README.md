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

## Project Overview & Solution:
A three-wheeled vehicle is modeled and is visualized in gazebo simulator and RViz. The world file provided is mapped using Simeltaneous Mapping and Loacalization (SLAM) method. For Path planning RRT* and A* algorithms were used on the map generated and PID algorithm is used for control algorithm. 

### Vehicle Modelling
The vehicle is modeled in an urdf file.

### Mapping the world file
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

### Path Planning
Path Planning gives the vehicle's trajectory from its starting state to some goal state in the map. There are most popularly two approaches for path planning - Search based methods & Sampling based methods. In Search based algorithms, cost to access each node from start node in graph is calculated, to get the most optimal or shortest path to reach the goal node. Common examples being Dijkstra's and A* algorithms. As the size of map and dimensions increases they become computational expensive but still give us the shortest possible path.

<br>
In Sampling based algorithms, as the name suggests random sampling of nodes to get the path isn't done, instead a smart approach of Rapidly exploring Random Trees is used, which is often referred as basic RRT algorithm, it requires lesser computational power compared to Search Based Algorithms, but doesn't output the most optimal path for the robot. RRT* Algorithm( modification of RRT ) gives the nearest to most optimal path for the robot. 


<br>
The RRT* algorithm has been used for path planning. The code implementation takes the binary occupancy grid map as .npy file for detecting the obstacles and the start point, goal point and waypoints are given as input co-ordinates. The final path is visualized using the matplotlib library.
 
### Control Algorithm
PID algorithm is used

