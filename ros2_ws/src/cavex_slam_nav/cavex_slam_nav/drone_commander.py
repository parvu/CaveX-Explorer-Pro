#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')
        self.get_logger().info('CaveX Drone Commander Node Started.')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        
        # Subscriber for base station mode / detachment
        self.mode_sub = self.create_subscription(
            String,
            '/cavex/mode',
            self.mode_callback,
            10
        )
        self.current_mode = 'WALKING'
        
    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode == 'FLYING':
            self.get_logger().info('Detachment signal received! Initiating VTOL flight sequence.')
            self.initiate_takeoff()
            
    def initiate_takeoff(self):
        # Publish takeoff velocity
        twist = Twist()
        twist.linear.z = 1.0 # Ascend at 1 m/s
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Ascending into the vertical shaft...')

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
