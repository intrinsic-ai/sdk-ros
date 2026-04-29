#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_python_service_node')
    node.get_logger().info('Test Python service node started!')
    
    def callback(request, response):
        node.get_logger().info(f'Received request in Python: data={request.data}')
        response.success = True
        response.message = f'Processed data in Python: {request.data}'
        return response
        
    srv = node.create_service(SetBool, 'set_bool', callback)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
