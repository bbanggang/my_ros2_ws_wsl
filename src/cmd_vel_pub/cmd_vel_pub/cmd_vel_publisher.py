#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped


LINEAR_X  = 2.1
ANGULAR_Z = 1.0
PUBLISH_HZ = 10


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')

        qos = QoSProfile(depth=10)
        self.pub_ = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.timer_ = self.create_timer(1.0 / PUBLISH_HZ, self.timer_cb)

        self.get_logger().info(
            f'Publishing /cmd_vel  linear.x={LINEAR_X}  angular.z={ANGULAR_Z}  @ {PUBLISH_HZ}Hz'
        )

    def timer_cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x  = LINEAR_X
        msg.twist.linear.y  = 0.0
        msg.twist.linear.z  = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = ANGULAR_Z

        self.pub_.publish(msg)
        self.get_logger().info(
            f'Published  linear.x={msg.twist.linear.x}  angular.z={msg.twist.angular.z}',
            throttle_duration_sec=1.0
        )


def main():
    rclpy.init()
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
