#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
            qos_profile=qos_profile_sensor_data)

        self.timer_period = 0.1  # seconds
        self.time = self.create_timer(
            self.timer_period, 
            self.timer_callback)
        self.i = 0

        self.FRONT = 0
        self.RIGHT = 1
        self.BACK = 2
        self.LEFT = 3
        self.WALLDISTANCE  = 0.8

        self.g_range_ahead = [0.,0.,0.,0.]
        self.driving_forward = True
        self.state_change_time = 0
        self.get_logger().info('Init OK')

    def scan_callback(self,  msg):
        if( len(msg.ranges) > 0):
            self.g_range_ahead = [min(min(msg.ranges[0:44]),min(msg.ranges[315:359])), # FRONT
            min(msg.ranges[45:134]), # RIGHT 
            min(msg.ranges[135:224]), # BACK
            min(msg.ranges[225:314])] # LEFT

#            self.g_range_ahead = min(msg.ranges)
#        self.get_logger().info('range min: %0.1f' % self.g_range_ahead)


    def timer_callback(self):
        bchange = False
        if self.driving_forward:
            if ( self.g_range_ahead[self.FRONT] < self.WALLDISTANCE or self.timer_now() >= self.state_change_time ):
                self.driving_forward = False
                self.state_change_time = self.timer_now() + self.timer_duration(1)
                bchange = True
        else :
            if self.timer_now() >= self.state_change_time:
                self.driving_forward = True
                self.state_change_time = self.timer_now() +  self.timer_duration(30)
                bchange = True

        twist = Twist()
        if self.driving_forward:
            twist.linear.x = 0.3
        else:
            if (self.g_range_ahead[self.LEFT] > self.g_range_ahead[self.RIGHT]):
                if ( self.g_range_ahead[self.LEFT] > self.WALLDISTANCE ):
                    twist.angular.z = -0.3
                    twist.linear.x = -0.01
                else:
                    if (self.g_range_ahead[self.BACK] > self.WALLDISTANCE):
                        twist.linear.x = -0.3
                        twist.angular.z = 0.0
                    else :
                        twist.linear.x = -0.01
                        twist.angular.z = 0.0
            else  :
                if ( self.g_range_ahead[self.RIGHT] > self.WALLDISTANCE):
                    twist.angular.z = 0.3
                    twist.linear.x = -0.01
                else:
                    if (self.g_range_ahead[self.BACK] > self.WALLDISTANCE):
                        twist.linear.x = -0.3
                        twist.angular.z = 0.0
                    else :
                        twist.linear.x = -0.01
                        twist.angular.z = 0.0

        
        self.cmd_vel_pub.publish(twist)

        if bchange :
            self.get_logger().info('Publishing: "D %d S %0.1f A %0.1f RF %0.1f RL %0.1f RB %0.1f RR %0.1f"' 
            % (self.driving_forward,twist.linear.x,twist.angular.z,self.g_range_ahead[0],self.g_range_ahead[1],self.g_range_ahead[2],self.g_range_ahead[3] ))
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