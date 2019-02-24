import cv2
# import matplotlib.pyplot as plt
import numpy as np
import sys
from time import sleep
import pickle
import pprint

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

args = sys.argv
cap = cv2.VideoCapture(1)

if len(args) == 3:
    cols = int(args[1])
    rows = int(args[2])
else:
    cols = 4
    rows = 11

objp = np.zeros((cols*rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)

objpoints = [] # 3d points in real world space
imgpoints = [] # ed points in image plane

result_matrix = {}

count = 0
COUNT_MAX = 5

while True:
    sleep(1)
    re, frame = cap.read()

    frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners_temp = cv2.findCirclesGrid(gray, (cols, rows), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)

    # success find chessboard
    if ret:
        objpoints.append(objp)

        corners = cv2.cornerSubPix(gray,corners_temp,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)

        frame = cv2.drawChessboardCorners(frame, (cols, rows), corners, ret)
        print('corners shape: ', corners.shape)
        print(count)

        count += 1

    cv2.imshow('image', frame)

    if count > COUNT_MAX:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        result_matrix['ret'] = ret
        result_matrix['mtx'] = mtx
        result_matrix['dist'] = dist
        result_matrix['rvecs'] = rvecs
        result_matrix['tvecs'] = tvecs

        pprint.pprint(result_matrix)
        output = open('calibration/camera_matrix', 'wb')
        pickle.dump(result_matrix, output)
        output.close
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
