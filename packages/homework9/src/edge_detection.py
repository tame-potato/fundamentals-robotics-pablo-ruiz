#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class detect_edges():

    def __init__(self):

        self.sub_cropped = message_filters.Subscriber("/image_cropped", Image)
        self.sub_white = message_filters.Subscriber("/image_white_filter", Image)
        self.sub_yellow = message_filters.Subscriber("/image_yellow_filter", Image)

        self.ts = message_filters.TimeSynchronizer([self.sub_cropped, self.sub_white, self.sub_yellow], queue_size=10)
        self.ts.registerCallback(self.callback)
        
        self.pub = rospy.Publisher("image_lines_all", Image, queue_size=10)
        self.pub_white = rospy.Publisher("image_lines_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("image_lines_yellow", Image, queue_size=10)

        self.bridge = CvBridge()
        
    def callback(self, img_cropped, img_white, img_yellow):

        img_cropped_all = self.bridge.imgmsg_to_cv2(img_cropped)
        img_cropped_white = img_cropped_all.copy()
        img_cropped_yellow = img_cropped_all.copy()
        img_white = self.bridge.imgmsg_to_cv2(img_white)
        img_yellow = self.bridge.imgmsg_to_cv2(img_yellow)        
        
        img_edges = cv2.Canny(img_cropped_all, 100, 400)

        img_edges_white = cv2.bitwise_and(img_edges, img_edges, mask=img_white)
        img_edges_yellow = cv2.bitwise_and(img_edges,img_edges, mask=img_yellow)

        lines_white = cv2.HoughLinesP(img_edges_white, rho=1, theta=np.pi/180, threshold=5, minLineLength=1, maxLineGap=1)
        lines_yellow = cv2.HoughLinesP(img_edges_yellow, rho=1, theta=np.pi/180, threshold=5, minLineLength=1, maxLineGap=1)
        
        for i in lines_white:
            cv2.line(img_cropped_all, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (255,0,0), 2) 
            cv2.line(img_cropped_white, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (255,0,0), 2)

        for i in lines_yellow:
            cv2.line(img_cropped_all, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (0,0,255), 2)
            cv2.line(img_cropped_yellow, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (0,0,255), 2)

        img_cropped_all = self.bridge.cv2_to_imgmsg(img_cropped_all, "bgr8")
        img_cropped_white = self.bridge.cv2_to_imgmsg(img_cropped_white, "bgr8")
        img_cropped_yellow = self.bridge.cv2_to_imgmsg(img_cropped_yellow, "bgr8")
            
        self.pub.publish(img_cropped_all)
        self.pub_white.publish(img_cropped_white)        
        self.pub_yellow.publish(img_cropped_yellow)
        
if __name__ == "__main__":
    
    rospy.init_node("edge_detection")

    detect_edges()

    rospy.spin()

    
