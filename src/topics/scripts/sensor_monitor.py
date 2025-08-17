#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pboon09_custom_interface.msg import SensorData

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor_node')

        self.subscription = self.create_subscription(
            SensorData,
            '/sensor_data',
            self.subscriber_callback,
            10)
        
        self.temp_threshold = 30.0
        self.humidity_threshold = 70.0

        self.get_logger().info('Sensor Monitor Node has been started.')
        
    def subscriber_callback(self, msg):
        self.get_logger().info(f'Received: ID={msg.id}, Temp={msg.temperature:.2f}°C, Humidity={msg.humidity:.2f}%, Status={msg.status}')
        
        if msg.temperature > self.temp_threshold:
            self.get_logger().warn(f'High temperature alert: {msg.temperature:.2f}°C exceeds threshold {self.temp_threshold}°C')
        
        if msg.humidity > self.humidity_threshold:
            self.get_logger().warn(f'High humidity alert: {msg.humidity:.2f}% exceeds threshold {self.humidity_threshold}%')
        
        if msg.status == "warning":
            self.get_logger().warn(f'Sensor {msg.id} reports warning status')

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()