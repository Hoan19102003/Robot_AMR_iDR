#!/usr/bin/env python3
"""
IMU Calibration Trigger
This script publishes a 't' command to the 'imu_command' topic to trigger IMU calibration.
Make sure the imu_ethernet_publisher node is running before executing this script.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class ImuCalibrationTrigger(Node):
    def __init__(self):
        super().__init__('imu_calibration_trigger')
        
        # Create publisher for IMU commands
        self.command_pub = self.create_publisher(String, 'imu_command', 10)
        
        # Wait a moment for publisher to be ready
        self.get_logger().info("Waiting for publisher to initialize...")
        self.create_timer(0.5, self.send_calibration_command)
        
    def send_calibration_command(self):
        """Send the calibration command 't' to the IMU"""
        msg = String()
        msg.data = 't'
        
        self.command_pub.publish(msg)
        self.get_logger().info("📤 Sent calibration command 't' to IMU")
        self.get_logger().info("⏳ IMU should now be calibrating (takes ~3 seconds)...")
        
        # Shutdown after sending
        self.create_timer(0.5, self.shutdown_node)
    
    def shutdown_node(self):
        """Shutdown the node after command is sent"""
        self.get_logger().info("✅ Calibration command sent successfully. Shutting down...")
        rclpy.shutdown()


def main():
    rclpy.init()
    
    try:
        calibration_trigger = ImuCalibrationTrigger()
        rclpy.spin(calibration_trigger)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()