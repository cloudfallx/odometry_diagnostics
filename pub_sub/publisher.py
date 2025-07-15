#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class pub_odom(Node):

    def __init__(self):
        super().__init__("pub_odom")
        self.get_logger().info("Pub_Odom has been started.")

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_+=1

def main(args=None):
    rclpy.init(args=args)
    node = pub_odom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()