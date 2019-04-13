#!/usr/bin/env python
import rospy
import os
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
import calibration.calibrated as calibrated

maps = calibrated.getStereoMap()

class republishCalibrate():
    interporation = cv2.INTER_NEAREST

    def __init__(self):
        self.br = CvBridge()
        self.pub_left = rospy.Publisher('/left/image_calibrated', Image, queue_size=100)
        self.pub_right = rospy.Publisher('/right/image_calibrated', Image, queue_size=100)
        self.sub_left = message_filters.Subscriber('/left/image_decompressed', Image)
        self.sub_right = message_filters.Subscriber('/right/image_decompressed', Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([self.sub_left, self.sub_right], 100, 10.0)
        self.mf.registerCallback(self.callback)

    def calibrate(self, img_left, img_right):
        left = cv2.remap(img_left, maps['map1_l'], maps['map2_l'], interporation)
        right = cv2.remap(img_right, maps['map1_r'], maps['map2_r'], interporation)
        return left, right

    def callback(self, data_left, data_right):
        try:
           img_left = self.br.imgmsg_to_cv2(data_left, 'passthrough')
           img_right = self.br.imgmsg_to_cv2(data_right, 'passthrough')
        except CvBridgeError, e:
           rospy.logerr(e)

        remaped_left, remaped_right = self.calibrate(img_left, img_right)

        try:
            msg_left = self.br.cv2_to_imgmsg(img_left)
            msg_right = self.br.cv2_to_imgmsg(img_right)
        except CvBridgeError, e:
            rospy.logerr(e)
        
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

def main():
    rospy.init_node('republish_calibrate', anonymous=True)
    rc = republishCalibrate()
    rospy.spin()

if __name__ == '__main__':
    main()