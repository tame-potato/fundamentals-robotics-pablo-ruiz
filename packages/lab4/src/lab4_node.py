#! /usr/bin/env python3

import rospy
import math
import message_filters
from duckietown_msgs.msg import WheelEncoderStamped
from lab4.msg import Pose2D

class odometry:
    def __init__(self):

        self.sub_left = message_filters.Subscriber('left_wheel_encoder_node/tick', WheelEncoderStamped)
        self.sub_right = message_filters.Subscriber('right_wheel_encoder_node/tick', WheelEncoderStamped)
        self.ts = message_filters.TimeSynchronizer([self.sub_left, self.sub_right], queue_size=10)
        self.ts.registerCallback(self.callback)

        self.pub = rospy.Publisher('current_pose', Pose2D, queue_size=10)

        self.position = Pose2D()
        self.position.x = 0
        self.position.y = 0
        self.position.theta = 0

        self.distance = 0.1
        self.diameter = 0.068
        self.ticks_total = 135

    def delta(self,delta_r, delta_l):
        delta_s = (delta_r+delta_l) / 2
        delta_theta = (delta_r-delta_l) / (2*self.distance)
        return delta_s, delta_theta

    def ticks_to_m(self, ticks):
        return self.diameter * math.pi / self.ticks_total * ticks

    def callback(self, left_tick, right_tick):

        delta_r =  self.ticks_to_m(right_tick.data)
        delta_l =  self.ticks_to_m(left_tick.data)

        delta_s, delta_theta = self.delta(delta_r,delta_l)

        self.position.x += delta_s*math.cos(self.position.theta + (delta_theta/2))
        self.position.y += delta_s*math.sin(self.position.theta + (delta_theta/2))
        self.position.theta += delta_theta

        rospy.loginfo(self.position)
        self.pub.publish(self.position)

    
if __name__ == '__main__':

    rospy.init_node('odometry')

    odometry()

    rospy.spin()
