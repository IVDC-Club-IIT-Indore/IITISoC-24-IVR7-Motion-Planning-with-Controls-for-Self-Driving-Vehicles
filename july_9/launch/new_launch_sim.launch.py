import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'july_9' 
    # Adding the package directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Adding the pose and setting the sim time to true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Path to the custom world file
    world_file_name = 'modified_igvc.world'
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)

    # Include the Gazebo launch file, provided by the gazebo_ros package, with the custom world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_gazebo_ros, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_board_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_board_spawner,
        # Node(
        #     package='laser_republisher',
        #     executable='laser_republisher',
        #     name='laser_republisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )
    ])
