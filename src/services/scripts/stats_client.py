#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pboon09_custom_interface.srv import ComputeStats

class StatsClientInteractive(Node):
    def __init__(self):
        super().__init__('stats_client_node')
        self.client = self.create_client(ComputeStats, '/compute_statistics')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = ComputeStats.Request()

        self.get_logger().info('Statistics service client Node is ready')
        
    def send_request(self, numbers):
        self.request.numbers = numbers
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f'Sending {len(numbers)} numbers for statistics computation')
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = StatsClientInteractive()
    
    while True:
        try:
            print('\n--- Statistics Calculator ---')
            print('Enter numbers separated by spaces (or type "exit" to quit)')
            user_input = input('Numbers: ')
            
            if user_input.lower() == 'exit':
                print('Exiting...')
                break
            
            numbers = [float(x) for x in user_input.split()]
            
            if len(numbers) == 0:
                print('Please enter at least one number')
                continue
            
            future = node.send_request(numbers)
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    print(f'\n=== Statistics Results ===')
                    print(f'Numbers: {numbers}')
                    print(f'Count: {len(numbers)}')
                    print(f'Sum: {response.sum:.2f}')
                    print(f'Mean: {response.mean:.2f}')
                    print(f'Min: {response.min:.2f}')
                    print(f'Max: {response.max:.2f}')
                    print(f'Message: {response.message}')
                else:
                    print(f'Service failed: {response.message}')
            else:
                node.get_logger().error('Service call failed')
                
        except ValueError:
            print('Invalid input! Please enter valid numbers separated by spaces')
            continue
        except KeyboardInterrupt:
            print('\nExiting...')
            break
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()