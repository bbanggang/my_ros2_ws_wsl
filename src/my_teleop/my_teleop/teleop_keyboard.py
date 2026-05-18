#!/usr/bin/env python3

import os
import select
import sys

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TwistStamped

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# 최대 속도 제한
MAX_LIN_VEL = {
    'burger': 0.22,
    'waffle': 0.26,
    'waffle_pi': 0.26,
}
MAX_ANG_VEL = {
    'burger': 2.84,
    'waffle': 1.82,
    'waffle_pi': 1.82,
}

LIN_VEL_STEP = 0.01
ANG_VEL_STEP = 0.1

HELP_MSG = """
[ My Custom TurtleBot3 Teleop ]
-------------------------------
키보드 조작:
        w
   a    s    d
        x

  w / x  : 직진 속도 증가 / 감소
  a / d  : 회전 속도 증가 / 감소 (좌 / 우)
  스페이스 / s : 즉시 정지

CTRL-C : 종료
-------------------------------
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def clamp(value, low, high):
    return max(low, min(value, high))


def smooth(current, target, step):
    if target > current:
        return min(target, current + step)
    elif target < current:
        return max(target, current - step)
    return target


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('my_teleop_keyboard')

        self.ros_distro = os.environ.get('ROS_DISTRO', '')
        self.model = os.environ.get('TURTLEBOT3_MODEL', 'burger').lower()

        self.max_lin = MAX_LIN_VEL.get(self.model, MAX_LIN_VEL['burger'])
        self.max_ang = MAX_ANG_VEL.get(self.model, MAX_ANG_VEL['burger'])

        qos = QoSProfile(depth=10)
        if self.ros_distro == 'humble':
            self.pub = self.create_publisher(Twist, 'cmd_vel', qos)
        else:
            self.pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)

        self.target_lin = 0.0
        self.target_ang = 0.0
        self.control_lin = 0.0
        self.control_ang = 0.0

    def limit_lin(self, v):
        return clamp(v, -self.max_lin, self.max_lin)

    def limit_ang(self, v):
        return clamp(v, -self.max_ang, self.max_ang)

    def publish(self):
        self.control_lin = smooth(self.control_lin, self.target_lin, LIN_VEL_STEP / 2.0)
        self.control_ang = smooth(self.control_ang, self.target_ang, ANG_VEL_STEP / 2.0)

        if self.ros_distro == 'humble':
            msg = Twist()
            msg.linear.x = self.control_lin
            msg.angular.z = self.control_ang
            self.pub.publish(msg)
        else:
            msg = TwistStamped()
            msg.header.stamp = Clock().now().to_msg()
            msg.twist.linear.x = self.control_lin
            msg.twist.angular.z = self.control_ang
            self.pub.publish(msg)

    def stop(self):
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.control_lin = 0.0
        self.control_ang = 0.0
        self.publish()

    def print_status(self):
        print(f'  선속도: {self.target_lin:+.3f} m/s  |  각속도: {self.target_ang:+.3f} rad/s')


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopKeyboard()

    print(HELP_MSG)
    print(f'  모델: {node.model}  |  최대 직진: {node.max_lin} m/s  |  최대 회전: {node.max_ang} rad/s\n')

    status_counter = 0

    try:
        while True:
            key = get_key(settings)

            if key == 'w':
                node.target_lin = node.limit_lin(node.target_lin + LIN_VEL_STEP)
                status_counter += 1
                node.print_status()
            elif key == 'x':
                node.target_lin = node.limit_lin(node.target_lin - LIN_VEL_STEP)
                status_counter += 1
                node.print_status()
            elif key == 'a':
                node.target_ang = node.limit_ang(node.target_ang + ANG_VEL_STEP)
                status_counter += 1
                node.print_status()
            elif key == 'd':
                node.target_ang = node.limit_ang(node.target_ang - ANG_VEL_STEP)
                status_counter += 1
                node.print_status()
            elif key in (' ', 's'):
                node.target_lin = 0.0
                node.target_ang = 0.0
                node.control_lin = 0.0
                node.control_ang = 0.0
                node.print_status()
            elif key == '\x03':  # CTRL-C
                break

            # 20번마다 도움말 재출력
            if status_counter >= 20:
                print(HELP_MSG)
                status_counter = 0

            node.publish()

    except Exception as e:
        print(e)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
