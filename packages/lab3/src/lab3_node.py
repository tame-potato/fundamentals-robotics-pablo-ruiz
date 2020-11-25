#! /usr/bin/env python3

import rospy
from MODULES import PID_controller

class lane_following:

    def __init__(self):
        
        self.sub = rospy.Subscriber("/lane_pose",  , self.callback)
        self.pub = rospy.Publisher("",  , queu_size=10)
        self.controller1 = PID_controller()
        self.controller2 = PID_controller()
        self.controller1.gains_setter(1,1,1)
        self.controller2.gains_setter(1,1,1)

    def callback(self,data):

        self.dist_input = self.controller1.PID(data.d)
        self.orient_input = self.controller2.PID(data.phi)
        self.pub.publish()

if __name__ == "__main__":

    rospy.init_node("lab3")

    lane_following()

    rospy.spin()
