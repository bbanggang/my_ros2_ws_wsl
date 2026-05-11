import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32

class HelloworldPublisher(Node):
    def __init__(self):
        super().__init__('pub')
        qos_profile = QoSProfile(depth=10)
        self.pub = self.create_publisher(Int32, 'num', qos_profile)
        self.timer = self.create_timer(1, self.publish_helloworld_msg)
        self.count = 0

    def publish_helloworld_msg(self):
        msg = Int32()
        msg.data = self.count
        self.pub.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloworldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()