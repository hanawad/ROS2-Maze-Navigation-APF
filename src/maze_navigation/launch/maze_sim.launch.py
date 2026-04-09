import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_maze_nav = get_package_share_directory('maze_navigation')

    world_file = os.path.join(pkg_maze_nav, 'worlds', 'simple_maze.world')

    tb3_model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_burger', 'model.sdf')

    gz_sim_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen')

    start_robot_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-file', tb3_model_path,
            '-x',   '0.5',
            '-y',   '0.5',
            '-z',   '0.01',
            '-Y',   '0.0',
        ],
        output='screen')

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Sensor topics
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Command velocity (ROS2 → Gazebo)
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',

            # Wheel odometry — kept for reference but DO NOT use for navigation
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # ✅ Ground Truth pose directly from Gazebo physics engine
            # This is the TRUE position — unaffected by wheel slip or drift
            '/model/turtlebot3_burger/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gz_sim_cmd,
        start_robot_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
    ])