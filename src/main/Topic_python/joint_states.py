import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class EncoderJointStatePublisher(Node):
    def __init__(self, motor_left, motor_right):
        super().__init__('encoder_joint_state_node')
        self.motor_left = motor_left
        self.motor_right = motor_right

        # --- Cấu hình encoder ---
        self.ticks_per_rev = 10000        # theo datasheet EziServo
        self.gear_ratio = 0.2             # tỉ số truyền 1:5 (bánh quay chậm hơn 2 lần)
        self.left_angle = 0.0
        self.right_angle = 0.0

        # --- Đọc encoder ban đầu ---
        try:
            self.prev_left_ticks = self.motor_left.get_actual_position()
            self.prev_right_ticks = self.motor_right.get_actual_position()
            self.get_logger().info(
                f"✅ Encoder init: Left={self.prev_left_ticks}, Right={self.prev_right_ticks}"
            )
        except Exception as e:
            self.get_logger().error(f"⚠️ Lỗi đọc encoder ban đầu: {e}")
            # Giữ nguyên giá trị cũ thay vì reset về 0
            if not hasattr(self, 'prev_left_ticks'):
                self.prev_left_ticks = 0
            if not hasattr(self, 'prev_right_ticks'):
                self.prev_right_ticks = 0

        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # --- Publisher /joint_states ---
        self.pub_joint = self.create_publisher(JointState, 'joint_states', 10)

        # 🕒 Nên để tần số 10 Hz (0.1s)
        self.timer = self.create_timer(0.05, self.publish_joint_state)

    # --- Giải wrap khi encoder vượt giới hạn ---
    def unwrap_delta(self, delta_ticks):
        MAX_TICKS = self.ticks_per_rev
        if delta_ticks > MAX_TICKS / 2:
            delta_ticks -= MAX_TICKS
        elif delta_ticks < -MAX_TICKS / 2:
            delta_ticks += MAX_TICKS
        return delta_ticks

    def get_encoder_positions(self):
        try:
            left = self.motor_left.get_actual_position()
            right = self.motor_right.get_actual_position()
            return left, right
        except Exception as e:
            self.get_logger().error(f"⚠️ Lỗi đọc encoder: {e}")
            return None, None

    def publish_joint_state(self):
        current_left, current_right = self.get_encoder_positions()
        if current_left is None or current_right is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.001:
            return

        # --- Tính delta ticks ---
        delta_left = self.unwrap_delta(current_left - self.prev_left_ticks)
        delta_right = self.unwrap_delta(current_right - self.prev_right_ticks)

        # # --- Nếu bánh phải quay ngược khi robot tiến, đảo dấu ---
        delta_right = -delta_right

        # --- Chuyển đổi sang góc (radian) ---
        delta_left_angle = 2 * math.pi * delta_left / self.ticks_per_rev * self.gear_ratio
        delta_right_angle = 2 * math.pi * delta_right / self.ticks_per_rev * self.gear_ratio

        # --- Cập nhật góc tích lũy ---
        self.left_angle += delta_left_angle
        self.right_angle += delta_right_angle

        # --- Tính vận tốc góc ---
        left_vel = delta_left_angle / dt
        right_vel = delta_right_angle / dt

        # --- Lưu lại vị trí encoder ---
        self.prev_left_ticks = current_left
        self.prev_right_ticks = current_right

        # --- Gửi JointState ---
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left_joint', 'wheel_right_joint']
        msg.header.frame_id = "base_link"
        msg.position = [self.left_angle, self.right_angle]
        msg.velocity = [left_vel, right_vel]
        self.pub_joint.publish(msg)

        # --- Giới hạn log mỗi 5 giây ---
        if not hasattr(self, 'last_log_time'):
            self.last_log_time = 0.0
        if now - self.last_log_time > 5.0:
            self.last_log_time = now
            # self.get_logger().info(
            #     f"Left angle={self.left_angle:.4f} rad, vel={left_vel:.4f} rad/s | "
            #     f"Right angle={self.right_angle:.4f} rad, vel={right_vel:.4f} rad/s"
            # )



def main(args=None):
    from Library.ezi_servo_controller import EziServoController

    rclpy.init(args=args)

    try:
        motor_left = EziServoController("192.168.0.2", 2002)
        motor_right = EziServoController("192.168.0.7", 2001)
    except Exception as e:
        print(f"Lỗi kết nối EziServo: {e}")
        rclpy.shutdown()
        return

    node = EncoderJointStatePublisher(motor_left, motor_right)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧹 Dừng node bằng Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Node encoder_joint_state_node đã tắt an toàn.")


if __name__ == '__main__':
    main()