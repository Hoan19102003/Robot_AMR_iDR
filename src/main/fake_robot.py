import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import time

class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot_node')


        # Robot config
        self.wheel_radius = 0.03  # 3cm
        self.wheel_base = 0.16    # khoảng cách 2 bánh

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel = 0.0
        self.right_wheel = 0.0

        # Velocity commands
        self.v_linear = 0.0
        self.v_angular = 0.0

        # Subscribers & Publishers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.update)  # 20Hz

        self.get_logger().info("Fake robot node started.")

    def cmd_callback(self, msg):
        self.v_linear = msg.linear.x
        self.v_angular = msg.angular.z

    def update(self):
        dt = 0.05

        # Kinematic differential drive
        v_l = self.v_linear - self.v_angular * self.wheel_base / 2
        v_r = self.v_linear + self.v_angular * self.wheel_base / 2

        # Update wheel rotation
        self.left_wheel += v_l / self.wheel_radius * dt
        self.right_wheel += v_r / self.wheel_radius * dt

        # Update robot pose
        self.x += self.v_linear * math.cos(self.theta) * dt
        self.y += self.v_linear * math.sin(self.theta) * dt
        self.theta += self.v_angular * dt

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        joint_state.position = [self.left_wheel, self.right_wheel]
        self.joint_pub.publish(joint_state)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.v_linear
        odom.twist.twist.angular.z = self.v_angular
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
