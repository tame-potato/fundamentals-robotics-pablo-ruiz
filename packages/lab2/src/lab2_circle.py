#! /usr/bin/env python3

# Lab 2 Circle
# Programmer: Pablo Ruiz

import rospy
from duckietown_msgs.msg import Twist2DStamped

class make_thing_do_circle:

    def __init__(self):

        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
        self.rate = rospy.Rate(10)

    def DO_CIRCLE_NOW(self):

        self.command = Twist2DStamped()

        self.command.v = 0.2

        self.command.omega = 1.1 

        for i in range(0, 150):

            self.pub.publish(self.command)

            self.rate.sleep()

        for i in range(0, 10):
            
            self.command.v = 0
        
            self.command.omega = 0

            self.pub.publish(self.command)

            self.rate.sleep()

if __name__ == "__main__":

    rospy.init_node("lab2")

    lab2 = make_thing_do_circle()

    lab2.DO_CIRCLE_NOW()

    

    

