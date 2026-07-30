#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class BaseStationController(Node):
    def __init__(self):
        super().__init__('base_station_controller')
        self.get_logger().info('CaveX Spot Base Station Node Started.')
        
        self.mode_pub = self.create_publisher(String, '/cavex/mode', 10)
        
        # Subscribe to odometry to know when we reach the end of the flooded zone (X=29.0)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/spot/odom',
            self.odom_callback,
            10
        )
        
        self.mode = 'WALKING'
        self.shaft_x_threshold = 29.0 # End of flooded section
        self.drone_deployed = False

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        
        if self.mode == 'WALKING' and current_x > -5.0:
            self.get_logger().info('Entering Flooded Zone. Switching to SAILING mode.')
            self.mode = 'SAILING'
            self.mode_pub.publish(String(data=self.mode))
            
        elif self.mode == 'SAILING' and current_x >= self.shaft_x_threshold and not self.drone_deployed:
            self.get_logger().info('Reached the end of Flooded Zone (X=29.0). Stopping base station and deploying drone.')
            self.mode = 'FLYING'
            self.mode_pub.publish(String(data=self.mode))
            self.drone_deployed = True

def main(args=None):
    rclpy.init(args=args)
    node = BaseStationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
