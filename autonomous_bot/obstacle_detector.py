#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class AutonomousExplorationNode(Node):
    def __init__(self):
        super().__init__('autonomous_exploration_node')

        # Subscriber to LaserScan
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize state variables
        self.turning = False
        self.turn_direction = -0.4  # Default right turn (negative z)

        self.get_logger().info("Autonomous Exploration Node Ready...")

    def laserscan_callback(self, msg):
        # Convert ranges to numpy array, ignoring inf values
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)

        # Define sectors using degrees (assuming 360 samples)
        sectors = {
            "Front": np.concatenate((ranges[0:15], ranges[-15:])),   # 0° ±15°
            "Left": ranges[60:120],                                  # 90° ±30°
            "Right": ranges[240:300],                                # 270° ±30°
        }

        # Minimum distance per sector
        min_distances = {name: np.min(sector) for name, sector in sectors.items()}

        # Thresholds
        stop_threshold = 0.25   # Immediate stop
        avoid_threshold = 0.5   # Obstacle avoidance

        # Create Twist message
        action = Twist()

        # Safety Stop
        if min_distances["Front"] < stop_threshold:
            action.linear.x = 0.0
            action.angular.z = self.turn_direction
            self.get_logger().warn("⚠️ Too close to wall! Stopping & turning away.")
        # Obstacle ahead
        elif min_distances["Front"] < avoid_threshold:
            action.linear.x = 0.0
            action.angular.z = self.turn_direction
            self.turning = True
            self.get_logger().info("🚧 Obstacle ahead, turning to avoid.")
        # Obstacle on left — turn right slightly
        elif min_distances["Left"] < avoid_threshold:
            action.linear.x = 0.1
            action.angular.z = -0.3
            self.get_logger().info("↩️ Obstacle on left, turning slightly right.")
        # Obstacle on right — turn left slightly
        elif min_distances["Right"] < avoid_threshold:
            action.linear.x = 0.1
            action.angular.z = 0.3
            self.get_logger().info("↪️ Obstacle on right, turning slightly left.")
        # Path is clear
        else:
            self.turning = False
            action.linear.x = 0.15
            action.angular.z = 0.0
            self.get_logger().info("✅ Path clear, moving forward.")


        self.publisher_.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
