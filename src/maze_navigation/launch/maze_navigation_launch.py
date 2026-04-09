"""
Topic bridge 

  /scan          gz → ROS 2   LaserScan   – obstacle detection for APF
  /odom          gz → ROS 2   Odometry    – wheel odometry (drift-prone fallback)
  /cmd_vel       ROS 2 → gz   TwistStamped – velocity commands from the planner
  /clock         gz → ROS 2   Clock       – sim time synchronisation
  /model/turtlebot3_burger/odometry
                 gz → ROS 2   Odometry    – ground-truth pose (no wheel-slip drift)
                                            requires OdometryPublisher plugin in SDF

ROS 2 distribution : Jazzy
Gazebo             : Harmonic (gz-sim 8)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Build and return the complete LaunchDescription for the maze simulation.

    Returns:
        LaunchDescription containing Gazebo, the robot spawner, and the
        topic bridge.
    """
    # Package directories
    pkg_maze_nav = get_package_share_directory('maze_navigation')
    pkg_tb3_gz   = get_package_share_directory('turtlebot3_gazebo')

    # File paths
    world_file = os.path.join(pkg_maze_nav, 'worlds', 'simple_maze.world')

    # Use the stock TurtleBot3 Burger SDF.
    # If you add the OdometryPublisher plugin (for ground-truth odometry),
    # copy the model into your package and point tb3_model_path there instead.
    tb3_model_path = os.path.join(
        pkg_tb3_gz, 'models', 'turtlebot3_burger', 'model.sdf'
    )

    # 1. Gazebo Sim – load the maze world and start the physics engine
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
    )

    # 2. Robot spawner – place TurtleBot3 Burger at the maze start position
    #    Start position: x=0.5, y=0.5 as specified in the project brief
    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-file', tb3_model_path,
            '-x',   '0.5',
            '-y',   '0.5',
            '-z',   '0.01',   # Slight Z offset to avoid ground-plane clipping
            '-Y',   '0.0',    # Yaw = 0 rad → robot faces positive X
        ],
        output='screen',
    )

    # 3. ros_gz_bridge – forward topics between Gazebo and ROS 2
    #
    #    Bridge notation:
    #      topic@ROS_type[gz_type   →  Gazebo publishes, ROS 2 subscribes
    #      topic@ROS_type]gz_type   →  ROS 2 publishes, Gazebo subscribes
    #
    #    Ground-truth odometry note:
    #      /model/turtlebot3_burger/odometry is only published by Gazebo
    #      when the gz::sim::systems::OdometryPublisher plugin is present in
    #      the robot SDF.  If the topic is silent, verify with:
    #          gz topic -l | grep turtlebot3_burger
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # LiDAR scan – used by the APF planner for obstacle detection
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # Wheel odometry – fallback pose source (subject to drift)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # Velocity commands – planner → Gazebo physics engine
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',

            # Simulation clock – keeps ROS 2 time in sync with Gazebo
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Ground-truth odometry – preferred pose source, drift-free.
            # Requires OdometryPublisher plugin in turtlebot3_burger/model.sdf.
            '/model/turtlebot3_burger/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    # Launch description
    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        gz_sim,
        robot_spawner,
        gz_bridge,
    ])