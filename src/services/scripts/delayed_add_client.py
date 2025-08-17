#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import asyncio

class AsyncioAddClientMulti(Node):
    def __init__(self):
        super().__init__('asyncio_add_client_node')
        self.client = self.create_client(AddTwoInts, '/delayed_add')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Asyncio add service client Node is ready')
        
    def send_request(self, delay, number):
        request = AddTwoInts.Request()
        request.a = delay
        request.b = number
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = AsyncioAddClientMulti()
    
    while True:
        try:
            print('\n--- Test Non-Blocking Service ---')
            print('1. Send single request')
            print('2. Send multiple requests (demonstrate non-blocking)')
            print('3. Exit')
            choice = input('Choice: ')
            
            if choice == '1':
                delay = int(input('Delay (seconds): '))
                number = int(input('Number to add: '))
                
                future = node.send_request(delay, number)
                print(f'Request sent (wait {delay}s for response)...')
                
                rclpy.spin_until_future_complete(node, future)
                
                if future.result() is not None:
                    response = future.result()
                    print(f'Result: {delay} + {number} = {response.sum}')
                    
            elif choice == '2':
                print('\nSending 3 requests with different delays:')
                
                future1 = node.send_request(5, 10)
                print('Request 1: delay=5s, number=10 (should return 15 after 5s)')
                
                future2 = node.send_request(2, 20)
                print('Request 2: delay=2s, number=20 (should return 22 after 2s)')
                
                future3 = node.send_request(3, 30)
                print('Request 3: delay=3s, number=30 (should return 33 after 3s)')
                
                print('\nWaiting for all responses (they should complete in order: 2, 3, 1)...\n')
                
                futures = [future1, future2, future3]
                labels = ['Request 1 (5s)', 'Request 2 (2s)', 'Request 3 (3s)']
                
                while futures:
                    for i, future in enumerate(futures):
                        if future.done():
                            if future.result() is not None:
                                print(f'{labels[i]} completed with result: {future.result().sum}')
                            futures.pop(i)
                            labels.pop(i)
                            break
                    rclpy.spin_once(node, timeout_sec=0.1)
                
                print('\nAll requests completed!')
                
            elif choice == '3':
                break
                
        except ValueError:
            print('Invalid input!')
        except KeyboardInterrupt:
            print('\nExiting...')
            break
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()