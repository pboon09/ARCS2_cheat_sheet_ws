#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pboon09_custom_interface.srv import ComputeStats

class StatsServer(Node):
    def __init__(self):
        super().__init__('stats_server_node')
        self.srv = self.create_service(ComputeStats, '/compute_statistics', self.compute_stats_callback)
        self.get_logger().info('Statistics service server is ready')

        self.get_logger().info('Statistics service server Node is ready')

    def compute_stats_callback(self, request, response):
        numbers = request.numbers
        
        if len(numbers) == 0:
            response.success = False
            response.message = 'Error: Empty array provided'
            response.mean = 0.0
            response.min = 0.0
            response.max = 0.0
            response.sum = 0.0
            self.get_logger().warn('Received empty array')
            return response
        
        response.sum = sum(numbers)
        response.mean = response.sum / len(numbers)
        response.min = min(numbers)
        response.max = max(numbers)
        response.success = True
        response.message = f'Successfully computed statistics for {len(numbers)} numbers'
        
        self.get_logger().info(f'Received {len(numbers)} numbers')
        self.get_logger().info(f'Computed: mean={response.mean:.2f}, min={response.min:.2f}, max={response.max:.2f}, sum={response.sum:.2f}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StatsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()