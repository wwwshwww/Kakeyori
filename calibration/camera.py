import cv2
import numpy as np
import sys
from time import sleep

class ChessCamera:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_chess_flag = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_FAST_CHECK
    calibrate_flag = 0

    def __init__(self, device_index, (cols, rows)):
        self.device_index = device_index
        self.cols = cols
        self.rows = rows
        self.cap = cv2.VideoCapture(self.device_index)
        self.objp = np.zeros((self.rows*self.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        self.size = self.getGray().reshape[::-1])

    def getCapture(self):
        return self.cap

    def getFrame(self):
        ret, frame = self.cap.read()
        return ret, frame

    def getGray(self):
        _, frame = self.getFrame()
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return self.gray

    def getCorners(self, frame):
        ret, corners = cv2.findChessboardCorners(frame, (self.cols, self.rows), flags=find_chess_flag)
        return ret, corners

    def appendPoints(self, corners):
        self.objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        self.imgpoints.append(corners)

    def getDrawnChess(self, frame, corners, ret):
        img = cv2.drawChessboardCorners(frame, (self.cols, self.rows), corners, ret)
        return img

    def getCalibrationInfo(self):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.size, None,None,flags=calibrate_flag)
        self.result['ret'] = ret
        self.result['mtx'] = mtx
        self.result['dist'] = dist
        self.result['rvecs'] = rvecs
        self.result['tvecs'] = tvecs
        return self.result
    
