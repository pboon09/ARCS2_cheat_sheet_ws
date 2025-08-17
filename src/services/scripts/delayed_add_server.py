#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import AddTwoInts
import asyncio

class AsyncioAddServer(Node):
    def __init__(self):
        super().__init__('asyncio_add_server_node')
        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            AddTwoInts, 
            '/delayed_add', 
            self.add_callback,
            callback_group=self.callback_group
        )
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Asyncio add service server is ready')
        self.get_logger().info('Timer will tick every 2 seconds to show non-blocking')
        self.request_counter = 0

        self.get_logger().info('Asyncio add service server Node is ready')
        
    def timer_callback(self):
        self.get_logger().info('⏰ Timer tick - Node is not blocked!')
        
    async def add_callback_async(self, request, response):
        delay_seconds = request.a
        number = request.b
        
        self.request_counter += 1
        req_id = self.request_counter
        
        self.get_logger().info(f'[Request #{req_id}] START: wait {delay_seconds}s, then add {delay_seconds} + {number}')
        
        if delay_seconds <= 0:
            delay_seconds = 1
        
        for i in range(int(delay_seconds)):
            self.get_logger().info(f'[Request #{req_id}] Counting: {i+1}/{delay_seconds}')
            await asyncio.sleep(1)
        
        response.sum = delay_seconds + number
        self.get_logger().info(f'[Request #{req_id}] DONE: {delay_seconds} + {number} = {response.sum}')
        
        return response
    
    def add_callback(self, request, response):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            return loop.run_until_complete(self.add_callback_async(request, response))
        finally:
            loop.close()

def main(args=None):
    rclpy.init(args=args)
    node = AsyncioAddServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()