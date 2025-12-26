#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import time

class ToggleJoyNode(Node):
    def __init__(self):
        super().__init__("toggle_joy_node")
        self.pub = self.create_publisher(Joy, "/joy", 10)

        # Khởi tạo pygame
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("Không tìm thấy joystick nào!")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Sử dụng joystick: {self.joystick.get_name()}")

        # Trạng thái toggle nút
        self.state_a = 0
        self.state_b = 0
        self.state_x = 0
        self.state_y = 0

        # Nút cũ
        self.prev_a = 0
        self.prev_b = 0
        self.prev_x = 0
        self.prev_y = 0

        # Thời gian debounce
        self.last_press_time = {'a':0.0,'b':0.0,'x':0.0,'y':0.0}
        self.debounce_time = 0.20  # 200ms

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def debounce(self, button_name):
        now = time.time()
        if now - self.last_press_time[button_name] >= self.debounce_time:
            self.last_press_time[button_name] = now
            return True
        return False

    def timer_callback(self):
        pygame.event.pump()

        # --- D-Pad ---
        hat_x, hat_y = self.joystick.get_hat(0)
        axes = [float(hat_x), float(hat_y)]  # trực tiếp, không cộng dồn

        # --- Nút A ---
        a = self.joystick.get_button(0)
        if a == 1 and self.prev_a == 0 and self.debounce("a"):
            self.state_a = 1 - self.state_a
        self.prev_a = a

        # --- Nút B ---
        b = self.joystick.get_button(1)
        if b == 1 and self.prev_b == 0 and self.debounce("b"):
            self.state_b = 1 - self.state_b
        self.prev_b = b

        # --- Nút X ---
        x_btn = self.joystick.get_button(3)
        if x_btn == 1 and self.prev_x == 0 and self.debounce("x"):
            self.state_x = 1 - self.state_x
        self.prev_x = x_btn

        # --- Nút Y ---
        y_btn = self.joystick.get_button(4)
        if y_btn == 1 and self.prev_y == 0 and self.debounce("y"):
            self.state_y = 1 - self.state_y
        self.prev_y = y_btn

        # --- Xuất Joy ---
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = [self.state_a, self.state_b, self.state_x, self.state_y]

        self.pub.publish(msg)
        self.get_logger().debug(f"Published Joy: axes={msg.axes}, buttons={msg.buttons}")

def main(args=None):
    rclpy.init(args=args)
    node = ToggleJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()