#!/usr/bin/env python3
"""
Test script to verify USB-CAN-A bridge functionality with turtle visualization.

This script demonstrates:
1. USB-CAN communication between two adapters
2. CANopen-like message simulation
3. Turtle visualization of motor feedback
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, String


class USBCANTester(Node):
    """Test node for USB-CAN bridge functionality."""

    def __init__(self):
        super().__init__('usb_can_tester')

        # Publishers for sending commands
        self.velocity_pub = self.create_publisher(
            Float64, 'target_velocity', 10)

        # Subscribers for monitoring status
        self.bridge_status_sub = self.create_subscription(
            String, 'bridge_status', self.bridge_status_callback, 10)
        self.motor_status_sub = self.create_subscription(
            String, 'motor_status', self.motor_status_callback, 10)
        self.can_tx_sub = self.create_subscription(
            String, 'can_tx_status', self.can_tx_callback, 10)
        self.can_rx_sub = self.create_subscription(
            String, 'can_rx_status', self.can_rx_callback, 10)

        # Test state
        self.test_phase = 0
        self.start_time = time.time()

        # Timer for test sequence
        self.test_timer = self.create_timer(2.0, self.run_test_sequence)

        self.get_logger().info("USB-CAN Tester started")
        self.get_logger().info("Monitoring bridge and motor status...")

    def bridge_status_callback(self, msg):
        """Handle bridge status messages."""
        self.get_logger().info(f"Bridge: {msg.data}")

    def motor_status_callback(self, msg):
        """Handle motor status messages."""
        self.get_logger().info(f"Motor: {msg.data}")

    def can_tx_callback(self, msg):
        """Handle CAN TX status messages."""
        self.get_logger().debug(f"CAN TX: {msg.data}")

    def can_rx_callback(self, msg):
        """Handle CAN RX status messages."""
        self.get_logger().debug(f"CAN RX: {msg.data}")

    def run_test_sequence(self):
        """Run automated test sequence."""
        current_time = time.time() - self.start_time

        # Test phases
        if self.test_phase == 0:
            self.get_logger().info("Phase 0: System startup - waiting for initialization")
            self.test_phase = 1

        elif self.test_phase == 1:
            self.get_logger().info("Phase 1: Testing zero velocity command")
            self.send_velocity_command(0.0)
            self.test_phase = 2

        elif self.test_phase == 2:
            self.get_logger().info("Phase 2: Testing positive velocity")
            self.send_velocity_command(2.0)
            self.test_phase = 3

        elif self.test_phase == 3:
            self.get_logger().info("Phase 3: Testing negative velocity")
            self.send_velocity_command(-1.5)
            self.test_phase = 4

        elif self.test_phase == 4:
            self.get_logger().info("Phase 4: Testing sine wave velocity profile")
            velocity = 3.0 * math.sin(current_time * 0.5)
            self.send_velocity_command(velocity)

            if current_time > 60:  # Run sine wave for 1 minute
                self.test_phase = 5

        elif self.test_phase == 5:
            self.get_logger().info("Phase 5: Test complete - stopping motor")
            self.send_velocity_command(0.0)
            self.test_phase = 6

        else:
            self.get_logger().info("Test sequence completed!")

    def send_velocity_command(self, velocity):
        """Send velocity command to the system."""
        msg = Float64()
        msg.data = velocity
        self.velocity_pub.publish(msg)
        self.get_logger().info(f"Sent velocity command: {velocity:.2f} rad/s")


def main():
    """Main function."""
    rclpy.init()

    tester = USBCANTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
