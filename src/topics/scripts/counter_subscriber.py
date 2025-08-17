#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class CounterSubscriber(Node):
    def __init__(self):
        super().__init__('counter_subscriber_node')

        self.subscription = self.create_subscription(
            Int32,
            '/counter',
            self.subscriber_callback,
            10)
        
        self.get_logger().info('Counter Subscriber Node has been started.')

    def subscriber_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        if msg.data % 2 == 0:
            self.get_logger().info(f'{msg.data} is even')
        else:
            self.get_logger().info(f'{msg.data} is odd')

def main(args=None):
    rclpy.init(args=args)
    node = CounterSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()