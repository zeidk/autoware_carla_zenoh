#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from seri_demo.vehicle_interface import VehicleCommander

def main(args=None):
    '''
    Main function showing a multi-threaded executor.
    '''
    rclpy.init(args=args)
    vehicle_node = VehicleCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(vehicle_node)
    try:
        vehicle_node.get_logger().info('Beginning demo, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        vehicle_node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    vehicle_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
