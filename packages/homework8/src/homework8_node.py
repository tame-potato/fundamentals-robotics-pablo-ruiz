#! /usr/bin/env python3

import rospy
import

class process_image():

    def __init__(self):
        self.sub = rospy.Subscriber("/image",   ,self.callback)
        self.pub = rospy.Publisher( "image_cropped", , queue_size=10)

    def callback(self):
        pass

if __name__ == "__main__":
    
    rospy.init_node("image_processing")

    process_image()

    rospy.spin()

    
