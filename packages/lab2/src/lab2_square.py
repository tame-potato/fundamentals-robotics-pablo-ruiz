#! /usr/bin/env python3

# Lab 2 Circle
# Programmer: Pablo Ruiz

import rospy
from duckietown_msgs.msg import Twist2DStamped

class make_thing_do_square:

    def __init__(self):

        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
        self.rate = rospy.Rate(10)
        self.command = Twist2DStamped()

    def DO_SQUARE_NOW(self):

        for n in range(0,4):
            self.command.omega = 0
            self.command.v = 0.3
            
            for i in range(0, 40):

                self.pub.publish(self.command)

                self.rate.sleep()

            self.command.omega = 4
            self.command.v = 0

            for i in range(0, 4):            

                self.pub.publish(self.command)

                self.rate.sleep()

        self.command.omega = 0
        self.command.v = 0

        for n in range(0,10):

            self.pub.publish(self.command)

            self.rate.sleep()


if __name__ == "__main__":

    rospy.init_node("lab2")

    lab2 = make_thing_do_square()

    lab2.DO_SQUARE_NOW()

    

    

