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
        self.timer_ = self.f.commands.motion.go_forward_time
        self.loop_ = self.create_timer(self.timer_, self.__loop_callback_)

    def __generate_twist_message(self):
        msg = Twist()
        msg.linear.x = random.uniform(0, 1)
        msg.linear.y = random.uniform(0, 1)
        msg.linear.z = random.uniform(0, 1)
        msg.angular.x = random.uniform(0, 1)
        msg.angular.y = random.uniform(0, 1)
        msg.angular.z = random.uniform(0, 1)
        return msg

    def __loop_callback_(self):
        msg = self.__generate_twist_message()
        self.f.broker.send(
            self.f.topics.motion,
            self.f.commands.motion.go_forward,
            self.f.types.twist,
            msg,
        )
