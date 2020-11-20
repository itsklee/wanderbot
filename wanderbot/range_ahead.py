#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class Range_ahead(Node):

    def __init__(self):
        super().__init__('range_ahead')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Init OK')


    def scan_callback(self,  msg):
        self.get_logger().info('scan callback called')
        if( len(msg.ranges) > 0):
            self.range_ahead = msg.ranges[int(len(msg.ranges)/2)]
            self.get_logger().info('range ahead: %0.1f' % self.range_ahead)
        else:
            self.get_logger().info('msg.angle_min %0.1f msg.angle_max %0.1f msg.angle_increment %0.1f \
msg.time_increment %0.1f msg.scan_time %0.1f msg.range_min %0.1f msg.range_max %0.1f ' % (msg.angle_min,msg.angle_max,msg.angle_increment,
            msg.time_increment,msg.scan_time,msg.range_min,msg.range_max))


def main(args=None):
    rclpy.init(args=args)

    range_ahead = Range_ahead()

    rclpy.spin(range_ahead)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    range_ahead.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


