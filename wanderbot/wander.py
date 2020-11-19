#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Wander(Node):

    def __init__(self):
        super().__init__('wander')
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.timer_period = 0.1  # seconds
        self.time = self.create_timer(
            self.timer_period, 
            self.timer_callback)
        self.i = 0

        self.g_range_ahead = 1
        self.driving_forward = True
        self.state_change_time = self.timer_duration(5)
        self.get_logger().info('Init OK')

    def scan_callback(self,  msg):
        if( len(msg.ranges) > 0):
            self.g_range_ahead = min(msg.ranges)
        self.get_logger().info('range min: %0.1f' % self.g_range_ahead)


    def timer_callback(self):
        if self.driving_forward:
            if ( self.g_range_ahead < 0.8 or self.timer_now() >= self.state_change_time):
                self.driving_forward = False
                self.state_change_time = self.timer_now() + self.timer_duration(5)
        else :
            if self.timer_now() >= self.state_change_time:
                self.driving_forward = True
                self.state_change_time = self.timer_now() +  self.timer_duration(10)

        twist = Twist()
        if self.driving_forward:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0.5
        
        self.cmd_vel_pub.publish(twist)
              
        self.get_logger().info('Publishing: "Direction %d  Time %d Speed %0.1f Angle %0.1f"' 
        % (self.driving_forward,self.i,twist.linear.x,twist.angular.z))
        self.i += 1

    def timer_now(self):
        return self.i

    def timer_duration(self,dura):
        return 1. / self.timer_period * dura


def main(args=None):
    rclpy.init(args=args)

    wander = Wander()

    rclpy.spin(wander)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()