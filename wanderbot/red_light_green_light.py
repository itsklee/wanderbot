#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Red_light_green_light_Publisher(Node):

    def __init__(self):
        super().__init__('red_light_green_light')
        self.cmd_vel_pub = self.create_publisher(Twist,'turtle1/cmd_vel',1)

        self.timer_period = 0.1  # seconds
        self.time = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

        self.red_light_twist = Twist()
        self.green_light_twist = Twist()
        self.green_light_twist.linear.x = 0.5

        self.driving_forward = False
        self.light_change_time = self.timer_duration(3)


    def timer_callback(self):

        if self.driving_forward:
            self.cmd_vel_pub.publish(self.green_light_twist)
        else:
            self.cmd_vel_pub.publish(self.red_light_twist)

        if self.light_change_time  <= self.timer_now():
            self.driving_forward = not self.driving_forward
            self.light_change_time = self.timer_now() + self.timer_duration(3)

        self.get_logger().info('Publishing: "Direction %d  Time %d"' % (self.driving_forward,self.i))
        self.i += 1

    def timer_now(self):
        return self.i

    def timer_duration(self,dura):
        return 1. / self.timer_period * dura

        


def main(args=None):
    rclpy.init(args=args)

    red_light_green_light_publisher = Red_light_green_light_Publisher()


    rclpy.spin(red_light_green_light_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    red_light_green_light_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()