#! usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from odometry_hw.msg import Pose2D
from odometry_hw.msg import DistWheel

class hw7:

    def __init__(self):
        self.pub = rospy.Publisher("pose", Pose2D, queue_size=10)
        self.sub = rospy.Subscriber("dist_wheel", DistWheel,self.callback)
        self.position = Pose2D()
        self.position.x	= 0
        self.position.y	= 0
        self.position.theta = 0
        self.L = 0.05
        self.delta_s = 0
        self.delta_theta = 0
        
    def delta(self, self.delta_r, self.delta_l, self.L):
        self.delta_s = (self.delta_r+self.delta_l) / 2
        self.delta_theta = (self.delta_r-self.delta_l) / (2*self.L)
        
    def callback(self, wheel_data):
        self.delta_r = wheel_data.dist_wheel_right
        self.delta_l =  wheel_data.dist_wheel_left

        self.delta(self.delta_r, self.delta_l, self.L)

        self.position.x += self.delta_s*math.cos(self.position.theta + (self.delta_theta/2))
        self.position.y += self.delta_s*math.sin(self.position.theta + (self.delta_theta/2))
        self.position.theta += self.delta_theta

        self.pub.publish(self.position)


if __name__ == "__main__":

    rospy.init_node("homework7")

    hw7()

    rospy.spin()

