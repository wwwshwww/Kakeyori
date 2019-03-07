import numpy as np
import cv2
import pickle
import pprint

import os
import sys

this_path = os.path.dirname(__file__)

def opener(file_path, massage):
    try:
        with open(this_path + '/' + file_path, 'rb') as f:
            data = pickle.load(f)
            pprint.pprint(data)
    except FileNotFoundError:
        print(message)
        exit()
    return data

def getCameraMatrix():
    return opener('camera_matrix', 'Camera matrix not fonud. Please run `calibration/realtimeCalibrate.py` before.')

def getWideResult():
    return opener('wide_result', 'info for wide camera not fonud. Please run `calibration/realtimeCalibrate_wide.py` before.')

def getUndistortedImage(image, cam_mtx, mode=0):
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx['mtx'], cam_mtx['dist'], (w,h), mode, (w,h))
    mapx, mapy = cv2.initUndistortRectifyMap(cam_mtx['mtx'], cam_mtx['dist'], None, newcameramtx, (w,h), 5)
    dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def getUndistortedImage_wide(image, K, D):
    h, w = image.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (h,w), cv2.CV_16SC2)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img