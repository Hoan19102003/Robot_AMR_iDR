#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import json
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
import time
class ImuEthernetPublisher(Node):
    def __init__(self, host='192.168.0.177', port=23):
        super().__init__('imu_ethernet_publisher')
        self.host = host
        self.port = port

        # Publisher IMU
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        # Subscriber lệnh
        self.command_sub = self.create_subscription(
            String,
            'imu_command',
            self.command_callback,
            10
        )

        self.yaw_offset = None
        self.buffer = ""
        self.send_queue = []        # danh sách lệnh cần gửi
        self.last_imu_msg = None    # lưu IMU gần nhất

        # Kết nối TCP tới IMU
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.host, self.port))
            self.get_logger().info(f"📡 Connected to IMU server at {self.host}:{self.port}")
            self.sock.send(b'\n')  # kích hoạt server gửi dữ liệu
        except Exception as e:
            self.get_logger().error(f"❌ Cannot connect to IMU server: {e}")
            raise

        # Timer đọc dữ liệu IMU 20 Hz
        self.timer = self.create_timer(0.05, self.read_imu)
        # Timer gửi lệnh TCP 20 Hz
        self.send_timer = self.create_timer(0.05, self.send_commands)
        
    def command_callback(self, msg):
        data = msg.data.lower()

        if data == "t":
            self.get_logger().info("Stopping IMU reading for calibration...")
            self.timer.cancel()

            self.send_queue.append(b"t\n")
            self.get_logger().info("Queued command: t")

            self.get_logger().info("Calibrating... wait 3 seconds...")

            # timer 1 lần
            self.resume_timer = self.create_timer(3.0, self.resume_imu_read)


    def resume_imu_read(self):
        self.get_logger().info("Resuming IMU reading...")
        self.resume_timer.cancel()
        
        # Clear the buffer to avoid parsing stale/partial data
        self.buffer = ""
        
        # Trigger IMU to start streaming
        try:
            self.sock.send(b'\n')
            self.get_logger().info("Triggered IMU to start streaming again")
        except Exception as e:
            self.get_logger().error(f"Failed to trigger IMU: {e}")
        
        # Wait a bit before starting to read
        time.sleep(0.1)
        
        # Restart the timer
        self.timer = self.create_timer(0.05, self.read_imu)



    def send_commands(self):
        """Gửi tất cả lệnh trong hàng đợi TCP tới IMU"""
        while self.send_queue:
            cmd = self.send_queue.pop(0)
            try:
                self.sock.send(cmd)
                self.get_logger().info(f"✅ Sent command: {cmd.decode().strip()}")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to send command: {e}")

    def read_imu(self):
        try:
            data = self.sock.recv(1024)
            if not data:
                self.get_logger().warn("⚠️ No data received from socket")
                return
            self.buffer += data.decode('utf-8', errors='ignore')
        except Exception as e:
            self.get_logger().error(f"Lỗi nhận dữ liệu từ IMU: {e}")
            return

        # Look for complete JSON objects (assuming they end with })
        while True:
            # Find the start and end of a JSON object
            start_idx = self.buffer.find('{')
            if start_idx == -1:
                # No JSON start found, clear garbage data before '{'
                self.buffer = ""
                return
            
            # Find matching closing brace
            end_idx = self.buffer.find('}', start_idx)
            if end_idx == -1:
                # Incomplete JSON, wait for more data
                # Keep only from '{' onwards
                self.buffer = self.buffer[start_idx:]
                
                # Prevent buffer from growing too large
                if len(self.buffer) > 2000:
                    self.get_logger().warn("⚠️ Buffer overflow, clearing")
                    self.buffer = ""
                return
            
            # Extract complete JSON object
            json_str = self.buffer[start_idx:end_idx + 1]
            
            # Remove processed data from buffer
            self.buffer = self.buffer[end_idx + 1:]
            
            # Try to parse the JSON
            try:
                data_json = json.loads(json_str)
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"⚠️ JSON parse error: {e}")
                continue  # Try next JSON object in buffer
            
            # Successfully parsed! Now process the IMU data
            self.process_imu_data(data_json)
            
            # Continue checking if there are more complete JSON objects in buffer

    def process_imu_data(self, data_json):
        """Process parsed IMU JSON data"""
        # Create message IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Theta từ IMU
        theta = float(data_json.get("theta", 0))
        imu_msg.orientation.z = theta

        # Quaternion
        imu_msg.orientation.x = float(data_json.get("orientation[x]", 0))
        imu_msg.orientation.y = float(data_json.get("orientation[y]", 0))
        imu_msg.orientation.w = float(data_json.get("orientation[w]", 1))

        # Angular velocity
        imu_msg.angular_velocity.x = float(data_json.get("angular_vel[x]", 0))
        imu_msg.angular_velocity.y = float(data_json.get("angular_vel[y]", 0))
        imu_msg.angular_velocity.z = float(data_json.get("angular_vel[z]", 0))

        # Linear acceleration
        imu_msg.linear_acceleration.x = float(data_json.get("linear_acc[x]", 0))
        imu_msg.linear_acceleration.y = float(data_json.get("linear_acc[y]", 0))
        imu_msg.linear_acceleration.z = float(data_json.get("linear_acc[z]", 0))

        # Tính yaw từ quaternion, reset lần đầu
        roll, pitch, yaw = euler_from_quaternion([
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ])
        if self.yaw_offset is None:
            self.yaw_offset = yaw
        yaw_corrected = yaw - self.yaw_offset

        # Debug
        print(f"[DEBUG IMU] Theta={theta:.4f}, yaw_corrected={yaw_corrected:.4f}")
        print(f"[DEBUG IMU] Quaternion x={imu_msg.orientation.x:.4f}, y={imu_msg.orientation.y:.4f}, "
            f"z={imu_msg.orientation.z:.4f}, w={imu_msg.orientation.w:.4f}")

        # Lưu IMU gần nhất
        self.last_imu_msg = imu_msg

        # Publish
        self.imu_pub.publish(imu_msg)

def main():
    rclpy.init()
    imu_node = ImuEthernetPublisher('192.168.0.177', 23)
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        print("\n🛑 Dừng chương trình")
    finally:
        imu_node.sock.close()
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
