#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class TimerPublisher(Node):
    def __init__(self):
        super().__init__('timer_publisher_node')
        self.declare_parameter('rate', 1.0)
        rate = self.get_parameter('rate').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(Int32, '/counter', 10)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info(f'Timer Publisher Node started with rate: {rate} Hz')
        
    def timer_callback(self):
        msg = Int32()
        msg.data = self.counter
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TimerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()