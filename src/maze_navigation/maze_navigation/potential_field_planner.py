#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math
import numpy as np

class MazePlanner(Node):
    def __init__(self):
        super().__init__('maze_planner')
        
        # Parameters
        self.goal_x = self.declare_parameter('goal_x', 9.0).value
        self.goal_y = self.declare_parameter('goal_y', 9.0).value
        self.k_att = 1.0
        self.k_rep = 50.0 # Adjusted for stability
        self.d_obs = 1.0
        
        # State variables
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.scan_data = None
        self.goal_reached = False
        
        # Stuck detection
        self.pos_history = []
        self.escape_mode = False
        self.escape_count = 0

        # Subscriptions and Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/model/turtlebot3_burger/odometry', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Maze Planner Started!")

    def odom_cb(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.curr_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_cb(self, msg):
        self.scan_data = np.array(msg.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def control_loop(self):
        if self.scan_data is None or self.goal_reached:
            return

        # Calculate distance to goal
        dx = self.goal_x - self.curr_x
        dy = self.goal_y - self.curr_y
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < 0.3:
            self.get_logger().info("Goal Reached!")
            self.goal_reached = True
            self.cmd_pub.publish(TwistStamped())
            return

        # 1. Attractive Force
        f_att_x, f_att_y = self.k_att * dx, self.k_att * dy

        # 2. Repulsive Force
        f_rep_x, f_rep_y = 0.0, 0.0
        for i, r in enumerate(self.scan_data):
            if r < self.d_obs and r > 0.05:
                angle = self.curr_yaw + self.angles[i]
                rep_mag = self.k_rep * (1.0/r - 1.0/self.d_obs) / (r**2)
                f_rep_x -= rep_mag * math.cos(angle)
                f_rep_y -= rep_mag * math.sin(angle)

        # 3. Total Force and Heading
        target_yaw = math.atan2(f_att_y + f_rep_y, f_att_x + f_rep_x)
        angle_err = math.atan2(math.sin(target_yaw - self.curr_yaw), math.cos(target_yaw - self.curr_yaw))

        # 4. Movement Logic
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        # Simple local minima escape
        self.pos_history.append((self.curr_x, self.curr_y))
        if len(self.pos_history) > 30:
            self.pos_history.pop(0)
            d = math.hypot(self.curr_x - self.pos_history[0][0], self.curr_y - self.pos_history[0][1])
            if d < 0.1: self.escape_mode = True

        if self.escape_mode:
            cmd.twist.angular.z = 0.5
            cmd.twist.linear.x = 0.05
            self.escape_count += 1
            if self.escape_count > 20:
                self.escape_mode = False
                self.escape_count = 0
                self.pos_history = []
        else:
            cmd.twist.linear.x = 0.15 * max(0.0, math.cos(angle_err))
            cmd.twist.angular.z = clip(1.5 * angle_err, -0.8, 0.8)

        self.cmd_pub.publish(cmd)

def clip(val, low, high):
    return max(low, min(val, high))

def main():
    rclpy.init()
    node = MazePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(TwistStamped())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()