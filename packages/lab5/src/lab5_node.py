#! /usr/bin/env python3

import rospy
import cv2
import system
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class process_image():

    def __init__(self):

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)

        self.image_size = (160,120)
        self.offset = 40

        self.pub_cropped = rospy.Publisher( "image_cropped", Image , queue_size=10)
        self.kernel_errode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        self.kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))

    def callback(self, rosImage):

        cvImage = self.bridge.compressed_imgmsg_to_cv2(rosImage)
        cvImage = cv2.resize(cvImage, self.image_size, interpolation=cv2.INTER_NEAREST)
        cvImage = new_image[self.offset:,:]
        cvImage_HSV = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)

        cvImage_filtered_white = cv2.inRange(cvImage_HSV, (0,0,150), (255,30,255))
        cvImage_filtered_yellow = cv2.inRange(cvImage_HSV, (30,100,150), (60,255,255))

        cvImage_filtered_white = cv2.erode(cvImage_filtered_white, self.kernel_errode)
        cvImage_filtered_yellow = cv2.erode(cvImage_filtered_yellow, self.kernel_errode)
        cvImage_filtered_white = cv2.dilate(cvImage_filtered_white, self.kernel_dilate)
        cvImage_filtered_yellow = cv2.dilate(cvImage_filtered_yellow, self.kernel_dilate)

        img_cropped_white = cvImage.copy()
        img_cropped_yellow = cvImage.copy()

        img_edges = cv2.Canny(cvImage, 100, 400)

        img_edges_white = cv2.bitwise_and(img_edges, img_edges, mask=cvImage_filtered_white)
        img_edges_yellow = cv2.bitwise_and(img_edges,img_edges, mask=cvImage_filtered_yellow)

        lines_white = cv2.HoughLinesP(img_edges_white, rho=1, theta=np.pi/180, threshold=5, minLineLe
ngth=1, maxLineGap=1)
        lines_yellow = cv2.HoughLinesP(img_edges_yellow, rho=1, theta=np.pi/180, threshold=5, minLine
Length=1, maxLineGap=1)

        for i in lines_white:
            cv2.line(cvImage, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (255,0,0), 2)
            cv2.line(img_cropped_white, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (255,0,0), 2)

        for i in lines_yellow:
            cv2.line(cvImage, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (0,0,255), 2)
            cv2.line(img_cropped_yellow, (i[0][0],i[0][1]), (i[0][2],i[0][3]), (0,0,255), 2)

        cvImage = self.bridge.cv2_to_imgmsg(img_cropped_all, "bgr8")
        img_cropped_white = self.bridge.cv2_to_imgmsg(img_cropped_white, "bgr8")
        img_cropped_yellow = self.bridge.cv2_to_imgmsg(img_cropped_yellow, "bgr8")

        self.pub.publish(img_cropped_all)
        self.pub_white.publish(img_cropped_white)
        self.pub_yellow.publish(img_cropped_yellow)

if __name__ == "__main__":

    rospy.init_node("edge_detection")

    detect_edges()

    rospy.spin()

