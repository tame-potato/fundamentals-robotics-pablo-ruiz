#! /usr/bin/env python3

import rospy
import math
import message_filters
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import Twist2DStamped
from lab4.msg import Pose2D_dist

class odometry:
    def __init__(self):

        self.sub_left = message_filters.Subscriber('left_wheel_encoder_node/tick', WheelEncoderStamped)
        self.sub_right = message_filters.Subscriber('right_wheel_encoder_node/tick', WheelEncoderStamped)
        self.sub = rospy.Subscriber('car_cmd_switch_node/cmd', Twist2DStamped, self.velocity_direction )

        self.ts = message_filters.TimeSynchronizer([self.sub_left, self.sub_right], queue_size=10)
        self.ts.registerCallback(self.callback)

        self.pub = rospy.Publisher('current_pose', Pose2D_dist, queue_size=10)

        self.pose = Pose2D_dist()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0
        self.pose.distance_travelled = 0
        self.previous_l_ticks = 0
        self.previous_r_ticks = 0
        self.sign_left = 1
        self.sign_right = 1

        self.axleSpacing = 0.1
        self.diameter = 0.068
        self.ticks_total = 135

    def delta(self,delta_r, delta_l):
        delta_s = (delta_r+delta_l) / 2
        delta_theta = (delta_r-delta_l) / (2*self.axleSpacing)
        return delta_s, delta_theta

    def ticks_to_m(self, ticks):
        return self.diameter * math.pi / self.ticks_total * ticks

    def velocity_direction(self, twist):

        if twist.v > 0:

            self.sign_right = 1
            self.sign_left = 1

        elif twist.v < 0:

            self.sign_right = -1
            self.sign_left = -1

        elif twist.omega > 0:

            self.sign_right = 1 
            self.sign_left = -1

        else:

            self.sign_right = -1 
            self.sign_left = 1

    def callback(self, full_left_tick, full_right_tick):

        rospy.logwarn("Right: "+str(full_right_tick.data))
        rospy.logwarn("Left: "+str(full_left_tick.data))

        if self.previous_l_ticks == 0 or self.previous_r_ticks == 0:

            self.previous_l_ticks = full_left_tick.data
            self.previous_r_ticks = full_right_tick.data

        else:

            left_tick = self.sign_left*(full_left_tick.data - self.previous_l_ticks)
            right_tick = self.sign_right*(full_right_tick.data - self.previous_r_ticks)

            delta_r =  self.ticks_to_m(right_tick)
            delta_l =  self.ticks_to_m(left_tick)

            delta_s, delta_theta = self.delta(delta_r,delta_l)

            self.pose.x += delta_s*math.cos(self.pose.theta + (delta_theta/2))
            self.pose.y += delta_s*math.sin(self.pose.theta + (delta_theta/2))
            self.pose.theta += delta_theta
            #self.pose.theta = (self.pose.theta / 2*math.pi) % 1 * (2*math.pi)

            self.pub.publish(self.pose)

            self.pose.distance_travelled += delta_s 
            self.previous_l_ticks = full_left_tick.data
            self.previous_r_ticks = full_right_tick.data
        
if __name__ == '__main__':

    rospy.init_node('odometry')

    odometry()

    rospy.spin()
