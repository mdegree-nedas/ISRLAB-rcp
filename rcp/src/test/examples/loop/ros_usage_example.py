import random
import time

from .ros.core import Freenove_4wd_smart_car

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from .controller import Controller


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
        # the decisor
        self.controller = Controller()
        self.f = Freenove_4wd_smart_car()
        # we will receive the sensor data here
        self.f.sensors.linetracker.callback = self.read_callback
        # every <timer> seconds will try to make a decision
        self.timer_ = self.f.commands.motion.go_forward_time
        self.loop_ = self.create_timer(self.timer_, self.__loop_callback_)
        # here we will have our sensor data
        self.sensor_data = None
        # we are ready to receive the sensors data
        self.f.broker.receive()


    def __loop_callback_(self):
        choice = self.controller.decision(self.sensor_data)

        if choice == 'left':
            msg = self.__generate_twist_message_left()
            self.get_logger().info('sending command turn_left\n')
            self.f.broker.send(
                self.f.topics.motion,
                self.f.commands.motion.turn_left,
                self.f.types.twist,
                msg,
            )
        elif choice == 'right':
            self.get_logger().info('sending command turn_right\n')
            msg = self.__generate_twist_message_right()
            self.f.broker.send(
                self.f.topics.motion,
                self.f.commands.motion.turn_right,
                self.f.types.twist,
                msg,
            )
        elif choice == 'forward':
            self.get_logger().info('sending command go_forward\n')
            msg = self.__generate_twist_message_forward()
            self.f.broker.send(
                self.f.topics.motion,
                self.f.commands.motion.go_forward,
                self.f.types.twist,
                msg,
            )


    def __generate_twist_message_forward(self):
        msg = Twist()
        msg.linear.x = random.uniform(0, 1)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg

    def __generate_twist_message_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = random.uniform(0, 1)
        return msg

    def __generate_twist_message_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -random.uniform(0, 1)
        return msg

    def read_callback(self, msg):
        self.sensor_data = msg["data"]
        self.get_logger().info("received sensor data: " + str(msg["data"]))
