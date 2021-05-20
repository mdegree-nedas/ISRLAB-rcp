import random
import time

from .ros.core import Freenove_4wd_smart_car

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


def main(args=None):
    rclpy.init(args=args)
    freenove = Freenove()
    rclpy.spin(freenove)
    freenove.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


class Freenove(Node):
    def __init__(self):
        super().__init__("freenove")
        self.f = Freenove_4wd_smart_car()
        self.f.sensors.linetracker.callback = self.read_callback
        self.f.broker.receive()

    def read_callback(self, msg):
        print(msg)
        self.get_logger().info("heard" + str(msg))
