import cv2
# import matplotlib.pyplot as plt
import numpy as np
import sys
from time import sleep
import pickle
import pprint

args = sys.argv
cap = cv2.VideoCapture(1)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

if len(args) == 3:
    cols = int(args[1])
    rows = int(args[2])
else:
    cols = 4
    rows = 11

objp = np.zeros((rows*cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)

objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane

result_matrix = {}

count = 0
COUNT_MAX = 40

while True:
    re, frame = cap.read()

    # frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flg = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_FAST_CHECK
    ret, corners = cv2.findChessboardCorners(frame, (cols, rows), flags=flg)
    # ret, corners = cv2.findCirclesGrid(gray, (cols, rows), flags=cv2.CALIB_CB_ASYMMETRIC_GRID+cv2.CALIB_CB_CLUSTERING)

    # success find chessboard
    if ret:
        objpoints.append(objp)

        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        imgpoints.append(corners)

        cv2.drawChessboardCorners(frame, (cols, rows), corners, ret)
        print(corners)
        print(count)

        count += 1

    cv2.imshow('image', frame)
    sleep(0.3)

    if count > COUNT_MAX:
        flag = 0
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None,flags=flag)
        result_matrix['ret'] = ret
        result_matrix['mtx'] = mtx
        result_matrix['dist'] = dist
        result_matrix['rvecs'] = rvecs
        result_matrix['tvecs'] = tvecs

        pprint.pprint(result_matrix)
        output = open('calibration/camera_matrix', 'wb')
        pickle.dump(result_matrix, output)
        output.close

        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error

        print(mean_error/len(objpoints))
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
