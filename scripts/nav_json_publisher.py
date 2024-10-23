#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from goby_ros_examples.msg import Nav
from std_msgs.msg import String

import json
from rosidl_runtime_py.convert import message_to_ordereddict

class NavJSONPublisher(Node):

    def __init__(self):
        super().__init__('nav_json_publisher')
        self.publisher_ = self.create_publisher(String, 'json_to_goby', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        ros_nav_msg = Nav()
        ros_nav_msg.lat = 47.5
        ros_nav_msg.lon = 112.3
        message_dict = message_to_ordereddict(ros_nav_msg)

        json_nav_msg = String()
        json_nav_msg.data = json.dumps(message_dict)
        
        self.publisher_.publish(json_nav_msg)
        self.get_logger().info('Publishing: "%s"' % (json_nav_msg.data))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = NavJSONPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
