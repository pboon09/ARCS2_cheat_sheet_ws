#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddClientInteractive(Node):
    def __init__(self):
        super().__init__('add_client_node')
        self.client = self.create_client(AddTwoInts, '/add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = AddTwoInts.Request()
        
        self.get_logger().info('Add service client Node is ready')
        
    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f'Sending request: a={a}, b={b}')
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = AddClientInteractive()
    
    while True:
        try:
            print('\n--- Add Two Integers Service ---')
            print('Enter two integers to add (or Ctrl+C to exit)')
            a = int(input('Enter first number: '))
            b = int(input('Enter second number: '))
            
            future = node.send_request(a, b)
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                response = future.result()
                print(f'Result: {a} + {b} = {response.sum}')
            else:
                node.get_logger().error('Service call failed')
                
        except KeyboardInterrupt:
            print('\nExiting...')
            break
        except ValueError:
            print('Please enter valid integers!')
            continue
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()