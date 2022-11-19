#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Move(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        
        timer_period = 0.5 # seconds (send a message 2x per second)
        self.timer = self.create_timer(timer_period, self.move)
        self.i = 0
    
    def move(self):
        # Create the velocity message
        vel_msg = Twist()
        vel_msg.linear.x = 1.0

        # Publish the velocity message each tick
        # so that our robot is continuously moving
        self.velocity_publisher.publish(vel_msg)
        
        # Logging
        self.get_logger().info(f'{self.i}: {vel_msg.linear}')
        self.i += 1
        

def main(args=None):
    rclpy.init(args=args)
    
    move = Move()
    rclpy.spin(move)
    
    move.destroy_node()
    rlcpy.shutdown()

if __name__ == '__main__':
    main()
