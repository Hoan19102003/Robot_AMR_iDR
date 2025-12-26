#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop_node')
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Giá trị hiện tại linear.x và angular.z
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Giới hạn
        self.max_lin = 1.0
        self.min_lin = -1.0
        self.max_ang = 1.0
        self.min_ang = -1.0

        # Chống double-step D-Pad
        self.prev_joy_y = 0
        self.prev_joy_x = 0

        # Step tăng/giảm mỗi lần nhấn
        self.step_linear = 0.1
        self.step_angular = 0.1

        self.get_logger().info("Joy teleop node started")

    def joy_callback(self, msg: Joy):
        joy_x = msg.axes[0]  # DPad trái/phải
        joy_y = msg.axes[1]  # DPad lên/xuống

        # Nút X (button 2 theo Xbox/PS) dừng khẩn cấp
        if msg.buttons[2] == 1:
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_twist.publish(twist)
            self.get_logger().info("⚠️ Emergency Stop pressed!")
            return  # không xử lý D-Pad khi nhấn X

        # --- Cập nhật giá trị nội bộ theo D-Pad ---
        if joy_y != 0 and self.prev_joy_y == 0:
            if joy_y > 0:
                self.current_linear_x += self.step_linear
            else:
                self.current_linear_x -= self.step_linear
            self.current_linear_x = max(self.min_lin, min(self.max_lin, self.current_linear_x))
        self.prev_joy_y = joy_y

        if joy_x != 0 and self.prev_joy_x == 0:
            if joy_x > 0:
                self.current_angular_z -= self.step_angular
            else:
                self.current_angular_z += self.step_angular
            self.current_angular_z = max(self.min_ang, min(self.max_ang, self.current_angular_z))
        self.prev_joy_x = joy_x

        # --- Nếu nhấn nút Y, mới publish cmd_vel ---
        if msg.buttons[3] == 1:  # nút Y
            twist = Twist()
            twist.linear.x = round(self.current_linear_x, 2)
            twist.angular.z = round(self.current_angular_z, 2)
            self.pub_twist.publish(twist)
            self.get_logger().info(
                f"📡 Cmd_vel sent: linear.x={twist.linear.x} angular.z={twist.angular.z}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()