#! /usr/bin/env python3

# Homework 3
# Programmer: Pablo Ruiz

import rospy
from std_msgs.msg import Float32

class hw3:

    def __init__(self):

        self.sub = rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("conversions", Float32, queue_size = 10)

    def conversion(self,feet,output_unit):

        switcher = {
            "meters": lambda a: a.data*0.3048 ,
            "smoots": lambda a: a.data*0.179104
        }

        func = switcher.get(output_unit,lambda: print("Unit Conversion Not Supported"))

        return func(feet)
        
    def callback(self, feet):

        if rospy.has_param("output_unit"):
            self.output_unit = rospy.get_param("output_unit")
        else:
            self.output_unit = "meters"
            
        self.pub.publish(self.conversion(feet,self.output_unit))

if __name__ == "__main__" :


    rospy.init_node("homework3")

    hw3()

    rospy.spin()

    
