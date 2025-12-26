#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json
import math
from tf_transformations import euler_from_quaternion

class ImuPublisher(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        super().__init__('imu_publisher')
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.ser.reset_input_buffer()
        self.yaw_offset = None
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz
        self.buffer = ""
        self.counter = 0
        self.get_logger().info(f"📡 Đang đọc dữ liệu IMU từ {port} @ {baudrate} bps")

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                self.buffer += data
        except serial.SerialException as e:
            self.get_logger().error(f"Lỗi serial: {e}")
            return

        while '{' in self.buffer and '}' in self.buffer:
            start = self.buffer.find('{')
            end = self.buffer.find('}', start)
            if end == -1:
                break
            json_str = self.buffer[start:end+1]
            self.buffer = self.buffer[end+1:]

            try:
                data = json.loads(json_str)
            except Exception as e:
                self.get_logger().warn(f"Lỗi parse IMU: {e} -> {json_str}")
                continue

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Theta đã tính sẵn từ IMU
            theta = float(data.get("theta", 0))

            # Gán vào orientation.z (hoặc angular_velocity.z nếu muốn)
            imu_msg.orientation.z = theta  

            # Quaternion chuẩn để các node khác dùng orientation
            imu_msg.orientation.x = float(data.get("orientation[x]", 0))
            imu_msg.orientation.y = float(data.get("orientation[y]", 0))
            imu_msg.orientation.w = float(data.get("orientation[w]", 1))


            imu_msg.angular_velocity.x = float(data.get("angular_vel[x]", 0))
            imu_msg.angular_velocity.y = float(data.get("angular_vel[y]", 0))
            imu_msg.angular_velocity.z = float(data.get("angular_vel[z]", 0))

            imu_msg.linear_acceleration.x = float(data.get("linear_acc[x]", 0))
            imu_msg.linear_acceleration.y = float(data.get("linear_acc[y]", 0))
            imu_msg.linear_acceleration.z = float(data.get("linear_acc[z]", 0))

            theta = float(data.get("theta", 0))

            roll, pitch, yaw = euler_from_quaternion([
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w
            ])
             # Reset yaw lần đầu
            if self.yaw_offset is None:
                self.yaw_offset = yaw
            yaw_corrected = yaw - self.yaw_offset

       
            # --- In tất cả các giá trị IMU để debug ---
            print(f"[DEBUG IMU] Quaternion: x={imu_msg.orientation.x:.4f}, y={imu_msg.orientation.y:.4f}, "
                f"z={imu_msg.orientation.z:.4f}, w={imu_msg.orientation.w:.4f}")
            print(f"[DEBUG IMU] Angular Vel: x={imu_msg.angular_velocity.x:.4f}, y={imu_msg.angular_velocity.y:.4f}, "
                f"z={imu_msg.angular_velocity.z:.4f}")
            print(f"[DEBUG IMU] Linear Acc: x={imu_msg.linear_acceleration.x:.4f}, "
                f"y={imu_msg.linear_acceleration.y:.4f}, z={imu_msg.linear_acceleration.z:.4f}")
            print(f"[DEBUG IMU] Theta: {theta:.4f}")
                
            self.imu_pub.publish(imu_msg)

def main():
    rclpy.init()
    imu_node = ImuPublisher('/dev/ttyUSB0', 115200)
    try:
        print("⚙️ Running IMU debug mode")
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình")
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()