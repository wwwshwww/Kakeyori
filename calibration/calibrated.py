import numpy as np
import cv2
import pickle
import pprint

import os
import sys

this_path = os.path.dirname(__file__)

def opener(file_path, massage):
    try:
        with open(this_path + '/temp/' + file_path, 'rb') as f:
            data = pickle.load(f)
            pprint.pprint(data)
    except FileNotFoundError:
        print(message)
        exit()
    return data

def getCameraMatrix():
    return opener('camera_matrix', 'Camera matrix not fonud. Please run `calibration/realtimeCalibrate.py` or `realtimeCalibrate_wide.py` before.')

def getUndistortedImage(image, cam_mtx, mode=0):
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx['mtx'], cam_mtx['dist'], (w,h), mode, (w,h))
    mapx, mapy = cv2.initUndistortRectifyMap(cam_mtx['mtx'], cam_mtx['dist'], None, newcameramtx, (w,h), 5)
    dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def getUndistortedImage_wide(image, cam_mtx):
    h, w = image.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(cam_mtx['mtx'], cam_mtx['dist'], np.eye(3), cam_mtx['mtx'], (w,h), cv2.CV_16SC2)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

# restration removed pixels by remap
def getUndistortedImage_wide_fixed(image, cam_mtx):
    balance = 0.8 # 1 is full
    h, w = image.shape[:2]
    K = cam_mtx['mtx']
    D = cam_mtx['dist']
    dim1 = (w, h)
    assert dim1[0]/dim1[1]

    dim2 = dim1
    dim3 = dim1

    scaled_K = K * dim1[0] / h
    scaled_K[2][2] = 1.0 # Except that K[2][2] is always 1.0

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return undistorted_img
