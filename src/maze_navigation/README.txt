=== Maze Navigation - ROS 2 Jazzy ===
Robot: TurtleBot3 Burger

Step 1: Source ROS 2
  source /opt/ros/jazzy/setup.bash

Step 2: Build workspace
  cd ~/ros2_project_ws
  colcon build --symlink-install

Step 3: Source workspace
  source install/setup.bash

Step 4: Launch simulation
  ros2 launch maze_navigation maze_sim.launch.py

Step 5: Run planner (in new terminal)
  source ~/ros2_project_ws/install/setup.bash
  ros2 run maze_navigation potential_field_planner \
    --ros-args -p goal_x:=9.0 -p goal_y:=9.0
