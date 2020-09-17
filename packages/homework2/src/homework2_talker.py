#! /usr/bin/env python3

# Homework 2 node
# Programmer: Pablo Ruiz        

                                                                                 
import rospy
from std_msgs.msg import Float32

class talker:
    def __init__(self, frequency):
        self.pub = rospy.Publisher("delta", Float32, queue_size=10)
        self.rate = rospy.Rate(frequency) #set message frequency

        self.message = 0

    def speak(self, step_size):
        self.message += step_size
        self.pub.publish(self.message)
        

if __name__ == '__main__':

    try:
    
        rospy.init_node('homework2')
        homework2 = talker(1)

        while not rospy.is_shutdown():
        
            homework2.speak(1)
            homework2.rate.sleep()

    except rospy.ROSInterruptException:
        pass


 


