import numpy as np
import cv2
import pickle
import pprint

import os
import sys

this_path = os.path.dirname(__file__)

def getCameraMatrix():
    try:
        with open(this_path + '/camera_matrix', 'rb') as f:
            cam_mtx = pickle.load(f)
            pprint.pprint(cam_mtx)
    except FileNotFoundError:
        print('Camera matrix not fonud. Please run `calibration/realtimeCalibrate.py` before.')
        exit()
    return cam_mtx

def calibration(image, cam_mtx, mode=0):
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx['mtx'], cam_mtx['dist'], (w,h), mode, (w,h))
    mapx, mapy = cv2.cv2.initUndistortRectifyMap(cam_mtx['mtx'], cam_mtx['dist'], None, newcameramtx, (w,h), 5)
    dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst
    