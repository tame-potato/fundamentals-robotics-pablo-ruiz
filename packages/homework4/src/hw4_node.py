#! /usr/bin/env python3

# Homework 3
# Programmer: Pablo Ruiz

import rospy
from std_msgs.msg import Float32
from homework4.msg import hw4_msg

class hw4:

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

        self.converted_output = hw4_msg()
        
        if rospy.has_param("output_unit"):
            self.converted_output.units = rospy.get_param("output_unit")
        else:
            self.converted_output.units = "meters"

        self.converted_output.data = self.conversion(feet,self.converted_output.units)

        self.pub.publish(self.converted_output.data)

if __name__ == "__main__" :


    rospy.init_node("homework3")

    hw4()

    rospy.spin()

    
