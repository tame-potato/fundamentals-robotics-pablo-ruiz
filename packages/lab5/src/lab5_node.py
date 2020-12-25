#! /usr/bin/env python3

import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList, Segment

class process_image():

    def __init__(self):

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)

        self.img_size = (160, 120)
        self.offset = 40

        self.pub = rospy.Publisher( "line_detector_node/segment_list", SegmentList , queue_size=10)
        self.pub_debug_img = rospy.Publisher("lab5_node/debug_img", Image, queue_size=1)
        self.pub_white = rospy.Publisher("lab5_node/debug_img_white", Image, queue_size=1)
        self.pub_yellow = rospy.Publisher("lab5_node/debug_img_yellow", Image, queue_size=1)

        #self.kernel_errode = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        self.kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))

    def plotLines(self, img, lines, color=(255,0,0)):

        for i in lines:
            cv2.line(img, (i[0][0],i[0][1]), (i[0][2],i[0][3]), color, 2)

        return img


    def to_segment_list(self, lines_white_norm, lines_yellow_norm):

        segment_list = []

        if len(lines_white_norm) > 0:
            for x1, y1, x2, y2 in lines_white_norm: 

                segment = Segment()
                segment.color = 0

                segment.pixels_normalized[0].x = x1
                segment.pixels_normalized[0].y = y1
                segment.pixels_normalized[1].x = x2
                segment.pixels_normalized[1].y = y2

                segment_list.append(segment)


        if len(lines_yellow_norm) > 0:
            for x1, y1, x2, y2 in lines_yellow_norm: 

                segment = Segment()
                segment.color = 0

                segment.pixels_normalized[0].x = x1
                segment.pixels_normalized[0].y = y1
                segment.pixels_normalized[1].x = x2
                segment.pixels_normalized[1].y = y2

                segment_list.append(segment)


        return segment_list


    def callback(self, rosImage):

        segment_list = SegmentList()

        cvImage = self.bridge.compressed_imgmsg_to_cv2(rosImage)
        cvImage = cv2.resize(cvImage, self.img_size, interpolation=cv2.INTER_NEAREST)
        cvImage = cvImage[self.offset:,:]
        cvImage_HSV = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)

        arr_cutoff = np.array([0, self.offset, 0, self.offset])
        arr_ratio = np.array([1. / self.img_size[1], 1. / self.img_size[0], 1. / self.img_size[1], 1. / self.img_size[0]])

        cvImage_filtered_white = cv2.inRange(cvImage_HSV, (35,0,100), (179,60,255))
        cvImage_filtered_yellow = cv2.inRange(cvImage_HSV, (29,100,100), (35,255,255))

        #cvImage_filtered_white = cv2.erode(cvImage_filtered_white, self.kernel_errode)
        #cvImage_filtered_yellow = cv2.erode(cvImage_filtered_yellow, self.kernel_errode)
        cvImage_filtered_white = cv2.dilate(cvImage_filtered_white, self.kernel_dilate)
        cvImage_filtered_yellow = cv2.dilate(cvImage_filtered_yellow, self.kernel_dilate)

        img_edges = cv2.Canny(cvImage, 100, 400)

        img_edges_white = cv2.bitwise_and(img_edges, img_edges, mask=cvImage_filtered_white)
        img_edges_yellow = cv2.bitwise_and(img_edges,img_edges, mask=cvImage_filtered_yellow)

        lines_white = cv2.HoughLinesP(img_edges_white, rho=1, theta=np.pi/180, threshold=5, minLineLength=1, maxLineGap=1)
        lines_yellow = cv2.HoughLinesP(img_edges_yellow, rho=1, theta=np.pi/180, threshold=5, minLineLength=1, maxLineGap=1)
     
        if lines_white is not None:

            cvImage = self.plotLines(cvImage, lines_white)

            lines_white = lines_white.reshape((-1, 4))
            lines_white_norm = np.multiply(np.add(lines_white,arr_cutoff), arr_ratio)

        else:
            lines_white_norm = []

        if lines_yellow is not None:

            cvImage = self.plotLines(cvImage, lines_yellow, color=(0,0,255))

            lines_yellow = lines_yellow.reshape((-1, 4))
            lines_yellow_norm = np.multiply(np.add(lines_yellow,arr_cutoff), arr_ratio)
        
        else:
            lines_yellow_norm = []

        segment_list.segments.extend(self.to_segment_list(lines_white_norm, lines_yellow_norm))
        
        cvImage = self.bridge.cv2_to_imgmsg(cvImage, "bgr8")

        cvImage_filtered_white = self.bridge.cv2_to_imgmsg(cvImage_filtered_white, "mono8")
        cvImage_filtered_yellow = self.bridge.cv2_to_imgmsg(cvImage_filtered_yellow, "mono8")

        self.pub.publish(segment_list)
        self.pub_debug_img.publish(cvImage)
        self.pub_white.publish(cvImage_filtered_white)
        self.pub_yellow.publish(cvImage_filtered_yellow)

        rospy.logwarn("PABLO'S LAB 5 CODE")

if __name__ == "__main__":

    rospy.init_node("edge_detection")

    process_image()

    rospy.spin()

