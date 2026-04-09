#!/usr/bin/env python3
"""
Autonomous maze navigation using the Artificial Potential Field (APF) method.

Robot   : TurtleBot3 Burger
ROS 2   : Jazzy
Gazebo  : Harmonic (Gazebo Sim 8)

The planner subscribes to LiDAR scans and odometry, computes attractive
forces toward the goal and repulsive forces away from obstacles, then
publishes velocity commands on /cmd_vel.

Ground-truth odometry from Gazebo is preferred over wheel odometry to
eliminate the position drift caused by wheel slip against maze walls.

Course  : Robot programming
"""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


# Constants
SAFE_RANGE: float = 10.0   # Replacement value for invalid LiDAR readings (m)
MIN_RANGE: float = 0.05    # Readings below this are treated as invalid (m)


class PotentialFieldPlanner(Node):
    """
    ROS 2 node that navigates a differential-drive robot through a maze using
    the Artificial Potential Field (APF) method.

    The node runs a 10 Hz control loop that:
      1. Computes an attractive force pulling the robot toward the goal.
      2. Computes repulsive forces from every LiDAR beam within d_obs.
      3. Derives a desired heading from the resultant force vector.
      4. Publishes a TwistStamped command on /cmd_vel.

    When the robot is stuck in a local minimum (displacement < STUCK_DIST
    over HISTORY_LEN control cycles), it executes a two-phase escape
    manoeuvre: slow in-place rotation followed by slow forward motion.
    """

    # Initialisation

    def __init__(self) -> None:
        super().__init__('potential_field_planner')

        self._declare_parameters()
        self._read_parameters()
        self._init_state()
        self._init_comms()

        self.get_logger().info(
            f'PotentialFieldPlanner ready. '
            f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | '
            f'k_att={self.k_att}  k_rep={self.k_rep}  d_obs={self.d_obs} m'
        )

    def _declare_parameters(self) -> None:
        """Declare all ROS 2 parameters with default values."""
        self.declare_parameter('goal_x',           9.0)
        self.declare_parameter('goal_y',           9.0)
        self.declare_parameter('k_att',            1.0)
        self.declare_parameter('k_rep',           80.0)
        self.declare_parameter('d_obs',            1.0)
        self.declare_parameter('max_linear_vel',   0.15)
        self.declare_parameter('max_angular_vel',  0.80)
        self.declare_parameter('goal_tolerance',   0.25)

    def _read_parameters(self) -> None:
        """Read parameter values into instance attributes."""
        self.goal_x           = self.get_parameter('goal_x').value
        self.goal_y           = self.get_parameter('goal_y').value
        self.k_att            = self.get_parameter('k_att').value
        self.k_rep            = self.get_parameter('k_rep').value
        self.d_obs            = self.get_parameter('d_obs').value
        self.max_linear_vel   = self.get_parameter('max_linear_vel').value
        self.max_angular_vel  = self.get_parameter('max_angular_vel').value
        self.goal_tolerance   = self.get_parameter('goal_tolerance').value

    def _init_state(self) -> None:
        """Initialise all runtime state variables."""
        # Pose estimate (updated by odometry callbacks) 
        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_yaw: float = 0.0

        #  Sensor data 
        self.scan_data: np.ndarray | None = None
        self.scan_angles: np.ndarray | None = None

        #  Navigation flags 
        self.goal_reached: bool = False
        self.using_ground_truth: bool = False   # True once GT odometry arrives

        #  Collision-avoidance threshold 
        self.COLLISION_DIST: float = 0.20   # Stop if front arc < this (m)

        #  Local-minima detection 
        self.pos_history: list[tuple[float, float]] = []
        self.HISTORY_LEN: int = 30           # Number of poses to keep
        self.STUCK_DIST: float = 0.08        # Displacement threshold (m)

        # Escape-manoeuvre state 
        self.escape_mode: bool = False
        self.escape_counter: int = 0
        self.escape_direction: float = 1.0   # +1 = CCW, -1 = CW
        self.escape_attempts: int = 0
        self.ESCAPE_DURATION: int = 40       # Control cycles per escape attempt

    def _init_comms(self) -> None:
        """Create publishers, subscribers, and the control-loop timer."""
        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Ground-truth pose from Gazebo (requires OdometryPublisher plugin in SDF
        # and a ros_gz_bridge entry for this topic)
        self.create_subscription(
            Odometry,
            '/model/turtlebot3_burger/odometry',
            self._gt_odom_callback,
            10,
        )

        # Wheel odometry – used only when ground-truth is unavailable
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )

        # LiDAR scan
        self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10,
        )

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)

    # Helpers

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """
        Extract the yaw (rotation about Z) from a ROS quaternion message.

        Args:
            q: geometry_msgs/Quaternion with fields x, y, z, w.

        Returns:
            Yaw angle in radians, range [-π, π].
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _build_twist(
        self,
        linear: float = 0.0,
        angular: float = 0.0,
    ) -> TwistStamped:
        """
        Build a stamped velocity command message.

        Args:
            linear:  Linear velocity along X in m/s.
            angular: Angular velocity about Z in rad/s.

        Returns:
            A populated TwistStamped ready to publish.
        """
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        return cmd

    def _stop_robot(self) -> None:
        """Publish a zero-velocity command to bring the robot to a full stop."""
        self.cmd_vel_pub.publish(self._build_twist())

    # Callbacks

    def _gt_odom_callback(self, msg: Odometry) -> None:
        """
        Update the pose estimate from Gazebo ground-truth odometry.

        Ground-truth data is read directly from the Gazebo physics engine via
        ros_gz_bridge, so it is free of wheel-slip drift.  Once a message
        arrives on this topic the fallback wheel-odometry callback becomes a
        no-op.

        Args:
            msg: nav_msgs/Odometry from /model/turtlebot3_burger/odometry.
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

        if not self.using_ground_truth:
            self.using_ground_truth = True
            self.get_logger().info(
                'Ground-truth odometry active — wheel-slip drift eliminated.'
            )

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Update the pose estimate from wheel odometry (fallback only).

        This callback is silently ignored once ground-truth data is available.
        It is retained so the node works in setups where the Gazebo
        OdometryPublisher plugin has not been configured.

        Args:
            msg: nav_msgs/Odometry from /odom.
        """
        if self.using_ground_truth:
            return

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Store and sanitise a LiDAR scan for use in the control loop.

        Invalid readings (inf, NaN, < MIN_RANGE) are replaced with SAFE_RANGE
        so they do not contribute to the repulsive force calculation or cause
        division-by-zero errors.

        Args:
            msg: sensor_msgs/LaserScan from /scan.
        """
        ranges = np.array(msg.ranges, dtype=np.float64)

        # Replace every flavour of invalid reading with a safe large distance
        ranges = np.where(np.isinf(ranges), SAFE_RANGE, ranges)
        ranges = np.where(np.isnan(ranges), SAFE_RANGE, ranges)
        ranges = np.where(ranges < MIN_RANGE, SAFE_RANGE, ranges)
        self.scan_data = np.clip(ranges, 0.0, SAFE_RANGE)

        # Compute beam angles once and cache them
        if self.scan_angles is None:
            self.scan_angles = np.linspace(
                msg.angle_min, msg.angle_max, len(msg.ranges)
            )

    # -----------------------------------------------------------------------
    # Safety checks
    # -----------------------------------------------------------------------

    def _imminent_collision(self) -> bool:
        """
        Detect an obstacle dangerously close in the robot's forward arc.

        Checks all LiDAR beams within ±60° of the robot's heading.  If any
        reading is closer than COLLISION_DIST the function returns True and
        the control loop will issue a brief back-up command instead of the
        normal APF command.  This prevents the robot from pressing against
        walls hard enough to cause wheel slip and odometry drift.

        Returns:
            True if a collision-level obstacle is detected, False otherwise.
        """
        if self.scan_data is None:
            return False

        front_mask = np.abs(self.scan_angles) < math.radians(60)
        return bool(np.any(self.scan_data[front_mask] < self.COLLISION_DIST))

    def _is_stuck(self) -> bool:
        """
        Detect whether the robot is trapped in a local minimum.

        The robot is considered stuck if its total displacement over the last
        HISTORY_LEN control cycles (≈ 3 seconds at 10 Hz) is less than
        STUCK_DIST metres.

        Returns:
            True when a local minimum is confirmed, False otherwise.
        """
        self.pos_history.append((self.current_x, self.current_y))
        if len(self.pos_history) > self.HISTORY_LEN:
            self.pos_history.pop(0)

        if len(self.pos_history) < self.HISTORY_LEN:
            return False

        dx = self.pos_history[-1][0] - self.pos_history[0][0]
        dy = self.pos_history[-1][1] - self.pos_history[0][1]
        return math.hypot(dx, dy) < self.STUCK_DIST

    # Local-minima escape

    def _escape_local_minima(self) -> None:
        """
        Execute one step of the two-phase escape manoeuvre.

        Detection condition (set by the caller):
            Robot displacement < STUCK_DIST over 3 seconds.

        Recovery sequence (ESCAPE_DURATION control cycles total):
            Phase 1 – first 40%: slow in-place rotation to break force
                      symmetry caused by symmetric obstacle arrangements.
            Phase 2 – remaining 60%: slow forward motion to leave the
                      potential-energy well.

        Low speeds are intentional: fast rotations generate wheel slip that
        corrupts the odometry even when ground-truth is not available.

        After ESCAPE_DURATION steps the escape flag is cleared, the position
        history is reset, and the robot resumes normal APF navigation.
        """
        progress = self.escape_counter / self.ESCAPE_DURATION

        if progress < 0.40:
            # Phase 1: rotate slowly to break symmetry
            cmd = self._build_twist(linear=0.0,
                                    angular=self.escape_direction * 0.6)
        else:
            # Phase 2: creep forward to exit the potential well
            cmd = self._build_twist(linear=0.10, angular=0.0)

        self.cmd_vel_pub.publish(cmd)
        self.escape_counter += 1

        if self.escape_counter >= self.ESCAPE_DURATION:
            self.escape_mode = False
            self.escape_counter = 0
            self.pos_history = []
            self.escape_attempts += 1
            self.escape_direction *= -1.0  # Alternate direction each attempt
            self.get_logger().info(
                f'Escape attempt #{self.escape_attempts} complete. '
                f'Resuming APF navigation.'
            )

    # APF force computation

    def _compute_attractive_force(
        self, dx: float, dy: float
    ) -> tuple[float, float]:
        """
        Compute the attractive force pulling the robot toward the goal.

        Uses a linear (conical) potential: F_att = k_att * (P_goal – P_robot).
        This guarantees the force is always directed toward the goal and
        grows proportionally with distance, preventing oscillation near the
        goal.

        Args:
            dx: Signed distance to goal along X (m).
            dy: Signed distance to goal along Y (m).

        Returns:
            Tuple (f_att_x, f_att_y) in Newtons (arbitrary units).
        """
        return self.k_att * dx, self.k_att * dy

    def _compute_repulsive_force(self) -> tuple[float, float]:
        """
        Compute the cumulative repulsive force from all nearby obstacles.

        For each LiDAR beam with range d ≤ d_obs:
            magnitude = k_rep * (1/d – 1/d_obs) / d²

        The force direction is opposite to the beam direction in world
        coordinates, pushing the robot away from the detected surface.
        Only beams within d_obs contribute; beams further away produce
        zero repulsive force (hard cut-off).

        Returns:
            Tuple (f_rep_x, f_rep_y) – sum of all repulsive contributions.
        """
        f_rep_x = 0.0
        f_rep_y = 0.0

        for i, d in enumerate(self.scan_data):
            if d > self.d_obs or d < MIN_RANGE:
                continue  # Outside influence zone or invalid

            # Transform beam angle from robot frame to world frame
            angle_world = self.current_yaw + self.scan_angles[i]

            # Gradient of the repulsive potential field
            magnitude = self.k_rep * (1.0 / d - 1.0 / self.d_obs) / (d * d)

            # Force is directed away from the obstacle (negative beam direction)
            f_rep_x -= magnitude * math.cos(angle_world)
            f_rep_y -= magnitude * math.sin(angle_world)

        return f_rep_x, f_rep_y

    # Main control loop
    

    def _control_loop(self) -> None:
        """
        Main 10 Hz control callback implementing the APF navigation loop.

        Execution order each cycle:
          1. Guard: wait for first sensor data.
          2. Guard: skip if goal already reached.
          3. Hard safety stop if collision is imminent.
          4. Local-minima detection; start escape if stuck.
          5. APF step: compute forces, desired heading, velocity commands.
          6. Publish velocity command.

        The position-source label ('GT' or 'ODOM') is included in every
        log message so drift issues are immediately visible during testing.
        """
        if self.scan_data is None:
            self.get_logger().warn(
                'Waiting for LiDAR data…',
                throttle_duration_sec=2.0,
            )
            return

        if self.goal_reached:
            return

        src = 'GT' if self.using_ground_truth else 'ODOM'

        # Hard safety stop 
        # Stop and back up if an obstacle is within the collision threshold.
        # This prevents wheel-on-wall contact, which is the primary cause of
        # odometry drift when ground-truth is not available.
        if self._imminent_collision() and not self.escape_mode:
            self.get_logger().warn(
                f'[{src}] Imminent collision at '
                f'({self.current_x:.2f}, {self.current_y:.2f}) — backing up.',
                throttle_duration_sec=1.0,
            )
            self.cmd_vel_pub.publish(
                self._build_twist(linear=-0.05,
                                  angular=self.escape_direction * 0.5)
            )
            return

        #  Local-minima detection 
        if not self.escape_mode and self._is_stuck():
            self.escape_mode = True
            self.escape_counter = 0
            self.escape_direction = random.choice([-1.0, 1.0])
            self.get_logger().warn(
                f'[{src}] Local minimum detected at '
                f'({self.current_x:.2f}, {self.current_y:.2f}) — '
                f'starting escape #{self.escape_attempts + 1}.'
            )

        if self.escape_mode:
            self._escape_local_minima()
            return

        #  Attractive force
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            self.get_logger().info(
                f'[{src}] Goal reached! '
                f'Final position: ({self.current_x:.3f}, {self.current_y:.3f}) | '
                f'Error: {dist:.3f} m'
            )
            self.goal_reached = True
            self._stop_robot()
            return

        f_att_x, f_att_y = self._compute_attractive_force(dx, dy)

        #  Repulsive force 
        f_rep_x, f_rep_y = self._compute_repulsive_force()

        # Resultant force → desired heading 
        f_total_x = f_att_x + f_rep_x
        f_total_y = f_att_y + f_rep_y

        desired_heading = math.atan2(f_total_y, f_total_x)

        # Normalise heading error to [-π, π]
        heading_error = math.atan2(
            math.sin(desired_heading - self.current_yaw),
            math.cos(desired_heading - self.current_yaw),
        )

        # Velocity mapping 
        # Linear velocity drops to zero when the heading error exceeds 90°,
        # ensuring the robot turns before moving forward.
        linear_vel = self.max_linear_vel * max(0.0, math.cos(heading_error))

        # Proportional angular control – gain of 1.2 gives responsive but
        # smooth turning without exciting wheel slip.
        angular_vel = float(
            np.clip(1.2 * heading_error,
                    -self.max_angular_vel,
                    self.max_angular_vel)
        )

        self.cmd_vel_pub.publish(
            self._build_twist(linear=linear_vel, angular=angular_vel)
        )

        self.get_logger().info(
            f'[{src}] pos=({self.current_x:.2f}, {self.current_y:.2f})  '
            f'dist={dist:.2f} m  '
            f'lin={linear_vel:.2f}  ang={angular_vel:.2f}',
            throttle_duration_sec=0.5,
        )


# Entry point

def main(args=None) -> None:
    """Initialise rclpy, spin the planner node, and clean up on exit."""
    rclpy.init(args=args)
    planner = PotentialFieldPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner._stop_robot()
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()