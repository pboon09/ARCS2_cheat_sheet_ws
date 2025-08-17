#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pboon09_custom_interface.msg import SensorData
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher_node')
        self.publisher_ = self.create_publisher(SensorData, '/sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sensor_id = 1001
        self.counter = 0

        self.get_logger().info('Sensor Publisher Node has been started.')
        
    def timer_callback(self):
        msg = SensorData()
        msg.id = self.sensor_id
        msg.temperature = 20.0 + random.uniform(-5.0, 15.0)
        msg.humidity = 50.0 + random.uniform(-20.0, 30.0)
        msg.status = "normal" if self.counter % 2 == 0 else "warning"
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ID={msg.id}, Temp={msg.temperature:.2f}, Humidity={msg.humidity:.2f}, Status={msg.status}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()