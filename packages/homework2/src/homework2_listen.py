#! /usr/bin/env python3

# Homework2_listen
# Programmer: Pablo Ruiz

import rospy
from std_msgs.msg import Float32

class listener:

    def __init__ (self):
        self.sub = rospy.Subscriber("total", Float32, self.callback)


    def callback (self, data):
        pass

if __name__ == '__main__' :


    rospy.init_node("listener")
    listener()


    rospy.spin()

