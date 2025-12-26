#!/usr/bin/env python3
import time
import math
import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster

from Library.ezi_servo_controller import EziServoController
from Topic_python.joint_states import EncoderJointStatePublisher
from Topic_python.imu_ethernet import ImuEthernetPublisher


class LocalizationController(Node):
    def __init__(self, motor_left, motor_right):
        super().__init__('localization_controller')

        self.motor_left = motor_left
        self.motor_right = motor_right

        # --- Robot parameters ---
        self.radius = 0.075
        self.wheel_distance = 0.5

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0          # Yaw integrated
        self.imu_yaw = 0.0     # Yaw from IMU

        self.prev_v = 0.0
        self.prev_w = 0.0
        self.cmd_threshold = 0.001

        self.prev_rpm_left = 0.0
        self.prev_rpm_right = 0.0
        self.prev_dir_left = None
        self.prev_dir_right = None

        self.last_encoder = None
        self.last_time = self.get_clock().now()

        self.lock = threading.Lock()

        # --- ROS ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.encoder_callback, 10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        self.create_timer(1.0, self.publish_static_imu_tf)

        self.get_logger().info("✅ LocalizationController started (IMU + encoder fusion)")

    # ============================================================
    # CMD_VEL
    # ============================================================
    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        if abs(v - self.prev_v) < self.cmd_threshold and abs(w - self.prev_w) < self.cmd_threshold:
            return

        self.prev_v = v
        self.prev_w = w

        v_left = v + (w * self.wheel_distance / 2.0)
        v_right = v - (w * self.wheel_distance / 2.0)

        rpm_left = (v_left / (2 * math.pi * self.radius)) * 60.0
        rpm_right = (v_right / (2 * math.pi * self.radius)) * 60.0

        dir_left = 1 if rpm_left > 0 else 0
        dir_right = 0 if rpm_right > 0 else 1

        if self.prev_dir_left != dir_left or abs(self.prev_rpm_left - rpm_left) > 1e-2:
            self.motor_left.move_velocity_rpm(abs(rpm_left), direction=dir_left)
            self.motor_left.velocity_override_rpm(abs(rpm_left))

        if self.prev_dir_right != dir_right or abs(self.prev_rpm_right - rpm_right) > 1e-2:
            self.motor_right.move_velocity_rpm(abs(rpm_right), direction=dir_right)
            self.motor_right.velocity_override_rpm(abs(rpm_right))

        self.prev_dir_left = dir_left
        self.prev_rpm_left = rpm_left
        self.prev_dir_right = dir_right
        self.prev_rpm_right = rpm_right

    # ============================================================
    # IMU CALLBACK
    # ============================================================
    def imu_callback(self, msg: Imu):
        # Chuyển quaternion -> yaw
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = yaw

    # ============================================================
    # STATIC TF: base_footprint -> imu_link
    # ============================================================
    def publish_static_imu_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'imu_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1

        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    # ============================================================
    # ENCODER CALLBACK → ODOM + TF
    # ============================================================
    def encoder_callback(self, msg: JointState):
        if self.last_encoder is None:
            self.last_encoder = msg.position
            self.last_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # --- Encoder distance ---
        d_left = (msg.position[1] - self.last_encoder[1]) * self.radius
        d_right = (msg.position[0] - self.last_encoder[0]) * self.radius

        self.last_encoder = msg.position
        self.last_time = current_time

        # Trung bình
        d = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / self.wheel_distance

        # --- Update state ---
        with self.lock:
            # Tích hợp yaw từ encoder
            self.th += dth

            # Fusion đơn giản với IMU (low-pass)
            alpha = 0.02
            self.th = (1 - alpha) * self.th + alpha * self.imu_yaw

            # Tích hợp position
            self.x += d * math.cos(self.th)
            self.y += d * math.sin(self.th)

        quat = quaternion_from_euler(0, 0, self.th)

        # --- TF ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = d / dt
        odom.twist.twist.angular.z = dth / dt

        self.odom_pub.publish(odom)


# ============================================================
# MAIN
# ============================================================
def main():
    rclpy.init()

    motor_left = EziServoController("192.168.0.2", 2002)
    motor_right = EziServoController("192.168.0.7", 2001)

    motor_left.set_servo_enable(True)
    motor_right.set_servo_enable(True)
    motor_left.clear_position()
    motor_right.clear_position()

    controller = LocalizationController(motor_left, motor_right)
    encoder_node = EncoderJointStatePublisher(motor_left, motor_right)
    imu_node = ImuEthernetPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(encoder_node)
    executor.add_node(imu_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motor_left.stop()
        motor_right.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
