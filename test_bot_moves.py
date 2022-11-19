#!/usr/bin/env python3

import math
import time
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('test_movement')
        
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        
        self.x = 0.0
        self.y = 0.0
        self.start_time = time.time()
        
        # We're expecting the robot to move this many meters from spawn
        self.expected_travel = 1.5
        
    def listener_callback(self, msg):
        # Uncomment for begugging only
        # self.get_logger().info(f'{msg.pose.pose.position}')
        
        # Save current X/Y position in the sim world
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Did we go the minimum distance?
        if self.get_distance_from_spawn() >= self.expected_travel:
            print(True)
            sys.exit(0) # Terminate gracefully
    
    def get_distance_from_spawn(self):
        """Use pythagoras to get total distance away from the spawn point
        """
        return math.sqrt(abs(self.x) ** 2 + abs(self.y) ** 2)

def main(args=None):
    rclpy.init(args=args)
    
    pos_subscriber = PositionSubscriber()
    rclpy.spin(pos_subscriber)
    
    pos_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
