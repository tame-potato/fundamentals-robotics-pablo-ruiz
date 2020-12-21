#! /usr/bin/env python3

import rospy
from MODULES import PID_controller
from duckietown_msgs.msg import LanePose, Twist2DStamped

class lane_following:

    def __init__(self):
        
        self.sub = rospy.Subscriber("lane_filter_node/lane_pose", LanePose , self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped , queue_size=10)
        self.controller1 = PID_controller()
        self.controller2 = PID_controller()
        self.controller1.gains_setter(5,0,0)
        self.controller2.gains_setter(5,0,0)

    def callback(self,data):

        output = Twist2DStamped()
        output.v = 0.2 ;

        dist = self.controller1.PID(data.d)
        orient = self.controller2.PID(data.phi)

        output.omega = dist+orient

        if output.omega > 8:
            output.omega = 8
        elif output.omega < -8:
            output.omega = -8

        rospy.logwarn("PABLO RUIZ LANE FOLLOWING CODE")

        rospy.logwarn(orient)
        rospy.logwarn(dist)
        rospy.logwarn(output)

        self.pub.publish(output)

if __name__ == "__main__":

    rospy.init_node("lab3")

    lane_following()

    rospy.spin()
