#!/usr/bin/env python
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
import calibration.calibrated as calibrated
import coordinate.intersection as intersection

import sys
from kakeyori_master.srv import *
import rospy

import cv2
import math

# STEREO_DIST = 650 # px dist of both lenz in real cammera system
MM_PER_PIX = 0.0048 # sensor size cm / pixel

class arucoFinder():
    def __init__(self):
        self.aruco = cv2.aruco
        self.ar_dict = self.aruco.getPredefinedDictionary(self.aruco.DICT_6X6_250)
        self.interporation = cv2.INTER_NEAREST

        self.w = 0
        self.h = 0

        self.stereo_mtx = calibrated.getStereoMatrix()
        self.camera1_mtx = self.stereo_mtx['K1']
        self.focal_length = self.camera1_mtx[0,0]
        self.camera1_trans = self.stereo_mtx['T']
        self.stereo_length = math.fabs(self.camera1_trans[0,0])

        self.br = CvBridge()
        self.setSubscriber()

        self.count = 0
        self.tmp_rw = 0
        self.tmp_r = 0
        self.tmp_th = 0
    
    def setSubscriber(self):
        self.sub_left = message_filters.Subscriber('/left/image_calibrated', Image)
        self.sub_right = message_filters.Subscriber('/right/image_calibrated', Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([self.sub_left, self.sub_right], 100, 10.0)
        self.mf.registerCallback(self.callback)

    def convertImgToCv2Both(self, data_left, data_right):
        try:
            img_left = self.br.imgmsg_to_cv2(data_left, 'rgb8')
            img_right = self.br.imgmsg_to_cv2(data_right, 'rgb8')
        except CvBridgeError, e:
            rospy.logger(e)
        return img_left, img_right

    def findAruco(self, img_left, img_right):
        corners_l, ids_l, rejectedImgPoints_l = self.aruco.detectMarkers(img_left, self.ar_dict)
        corners_r, ids_r, rejectedImgPoints_r = self.aruco.detectMarkers(img_right, self.ar_dict)
        return corners_l, corners_r, ids_l, ids_r, rejectedImgPoints_l, rejectedImgPoints_r

    def getQuadIntersections(self, corners_l, corners_r):
        inter_l = intersection.getQuadIntersection(corners_l[0][0][:])
        inter_r = intersection.getQuadIntersection(corners_r[0][0][:])
        return inter_l, inter_r

    def getRelative(self, inter_l, inter_r):
        normalize_angle = math.pi / 2
        secret = 2.2
        # dis = math.sqrt((inter_l[0] - inter_r[0])**2 + (inter_l[1] - inter_r[1])**2)
        dis = math.fabs(inter_l[0] - inter_r[0])
            # d = (X vec of T) * (a11 of K1) / disparity
        d = (self.stereo_length * self.focal_length) / dis * secret
        r = d

        pw = (inter_l[0] + inter_r[0]) / 2. - self.w / 2.
        ph = (inter_l[1] + inter_r[1]) / 2. - self.h / 2.

        print('pixel:', pw, ph)

        # calc relative coordinates X and Y
        rw = pw * d * MM_PER_PIX / 5
        rh = ph * d * MM_PER_PIX / 5

        # calc relative angle theta1 and theta2
        # temporary, only use theta1 that include angle of width
        # these are absolute angle of radian, so like to make to be normalized coordinates
        theta1 = math.atan2(r, rw) - normalize_angle # front is (1/2)*PI : 90 deg
        theta2 = math.atan2(r, rh) - normalize_angle # same

        print('rial:', rw, rh, r, math.degrees(theta1))

        return rw, rh, r, theta1, theta2
        
    def callback(self, data_left, data_right):
        img_left, img_right = self.convertImgToCv2Both(data_left, data_right)
        self.h, self.w = img_left.shape[:2]
        corners_l, corners_r, _, _, _, _ = self.findAruco(img_left, img_right)

        if len(corners_l) == 1 and len(corners_r) == 1:
            self.sub_left.unregister()
            self.sub_right.unregister()
            inter_l, inter_r = self.getQuadIntersections(corners_l, corners_r)
            rw, _, r, theta1, _ = self.getRelative(inter_l, inter_r)

            if self.count == 5:
                srw = self.tmp_rw / self.count
                sr = self.tmp_r / self.count
                sth = self.tmp_th / self.count
                print('heikin:', srw, sr, sth)
                isArrive = self.client(srw, sr, sth)
                
                print(isArrive, rospy.get_time())
                self.count = 0
                self.tmp_r = 0
                self.tmp_rw = 0
                self.tmp_th = 0

            elif self.count == 0:
                self.tmp_r = r
                self.tmp_rw = rw
                self.tmp_th = theta1
                self.count += 1
            else:
                if math.fabs(self.tmp_th / self.count - theta1) < 0.1:
                    self.tmp_r += r
                    self.tmp_rw += rw
                    self.tmp_th += theta1
                    self.count += 1
                else:
                    print("faild get available image.")

            print('get:', r, rw, theta1)

            self.setSubscriber()
            rospy.spin()

            
            
    def client(self, rw, r, theta):
        rospy.wait_for_service('point_surrenderer')
        try:
            point_surrenderer = rospy.ServiceProxy('point_surrenderer', RelativeCoordinates)
            resp = point_surrenderer(r, rw, theta)
            return resp.isGoal
        except rospy.ServiceException, e:
            print("Service call faild: %s"%e)

def main():
    rospy.init_node('investigator', anonymous=True)
    af = arucoFinder()
    rospy.spin()

if __name__ == "__main__":
    main()