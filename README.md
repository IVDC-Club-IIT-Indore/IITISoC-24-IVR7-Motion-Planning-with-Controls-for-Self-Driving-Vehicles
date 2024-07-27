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

### Control Algorithm
PID algorithm is used

