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
        self.LEFTSIDE = 4
        self.RIGHTSIDE = 5
        self.WALLDISTANCE  = 1
        self.turn_dir = self.LEFT

        self.twist = Twist()

        self.g_range_ahead = [0.,0.,0.,0.]
        self.g_range_ahead = [self.WALLDISTANCE*20,self.WALLDISTANCE*20,self.WALLDISTANCE*20,self.WALLDISTANCE*20]
        self.driving_forward = True
        self.bchange = False
        self.state_change_time = self.timer_duration(10)
        self.get_logger().info('Init OK')

    def timer_callback(self):
        self.do_wander()
        self.timer_increase()

    def scan_callback(self,  msg):
        if( len(msg.ranges) > 0):
            self.g_range_ahead = [
            min(min(msg.ranges[0:15]),min(msg.ranges[345:359])), # FRONT
            min(msg.ranges[255:285]), # RIGHT 
            min(msg.ranges[165:195]), # BACK
            min(msg.ranges[75:105]),  # LEFT
            min(msg.ranges[105:180]),  # LEFTSIDE
            min(msg.ranges[180:255])   # RIGHTSIDE
            ] 

    def set_state_change_time(self,duration):
        self.state_change_time = self.timer_now() + self.timer_duration(duration)
        self.bchange = True
        self.get_logger().info('Pub state_change_time %d' % duration)

    def check_change_direction_time(self):
        self.bchange = False
        if self.driving_forward:
            if (  not self.is_greater(self.FRONT,self.WALLDISTANCE) or
                 (self.timer_now() > self.state_change_time) ):
                self.driving_forward = False
                self.set_state_change_time(2)
        else :
            if self.timer_now() >= self.state_change_time:
                self.driving_forward = True
                self.set_state_change_time(10)

    def set_next_turn_dir(self):
        if self.is_greater(self.FRONT,self.WALLDISTANCE*2.0):
            if self.turn_dir == self.LEFT :
                self.turn_dir = self.RIGHT
            else:
                self.turn_dir = self.LEFT

    def change_forward_speed(self):
        if self.is_greater(self.FRONT,self.WALLDISTANCE): 
            self.set_twist(0.3,0.0)
        else :
            self.set_twist(0.01,0.0)

        if self.bchange:
            self.get_logger().info('Pub forward Speed %0.1f' % self.twist.linear.x)

    def move_turn(self,dir):
        if( dir == self.LEFT):
            self.set_twist(0.01,0.3)
            self.turn_dir = self.LEFT
            if self.bchange :
                self.get_logger().info('Pub LEFT LEFT')
        else:
            self.set_twist(0.01,-0.3)
            self.turn_dir = self.RIGHT
            if self.bchange :
                self.get_logger().info('Pub RIGHT RIGHT')

    def move_back(self,dir):
        if dir == self.LEFT :
            if self.is_greater(self.BACK,1.0):
                self.set_twist(-0.15,0.3)
            else :
                self.set_twist(0.01,0.3)
            if self.bchange :
                self.get_logger().info('Pub move_back LEFT')
        else :
            if self.is_greater(self.BACK,1.0):
                self.set_twist(-0.15,-0.3)
            else :
                self.set_twist(0.01,-0.3)
            if self.bchange :
                self.get_logger().info('Pub move_back RIGHT')

    def do_wander(self):
        self.check_change_direction_time()
        if self.driving_forward:
            self.change_forward_speed()
            self.set_next_turn_dir()
        else:
            if (self.turn_dir == self.LEFT):
                if (self.is_greater(self.LEFT,self.WALLDISTANCE) and 
                self.is_greater(self.RIGHTSIDE,self.WALLDISTANCE*0.2)):
                    self.move_turn(self.LEFT)
                else:
                    self.move_back(self.LEFT)
            else  : 
                if (self.is_greater(self.RIGHT,self.WALLDISTANCE) and 
                self.is_greater(self.LEFTSIDE,self.WALLDISTANCE*0.2)):
                    self.move_turn(self.RIGHT)
                else:
                    self.move_back(self.RIGHT)
        
        self.cmd_vel_pub.publish(self.twist)
        if self.bchange :
            self.get_logger().info('Pub"D %d TD %d T %d S %0.1f A %0.1f RF %0.1f RR %0.1f RB %0.1f RL %0.1f"' 
            % (self.driving_forward,self.turn_dir,self.i,
            self.twist.linear.x,self.twist.angular.z,
            self.g_range_ahead[0],self.g_range_ahead[1],self.g_range_ahead[2],self.g_range_ahead[3] ))

    def set_twist(self,speed,angle_velocity):
        self.twist.linear.x = speed
        self.twist.angular.z = angle_velocity

    def is_greater(self,dir,distance):
        if(dir <= len(self.g_range_ahead) and dir >= 0):
            if self.g_range_ahead[dir] > distance :
                return  True
            else:
                return False
        return False

    def timer_now(self):
        return self.i

    def timer_increase(self):
        self.i += 1

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