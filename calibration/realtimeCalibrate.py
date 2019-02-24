import cv2
# import matplotlib.pyplot as plt
import numpy as np
import sys
from time import sleep
import pickle
import pprint

args = sys.argv
cap = cv2.VideoCapture(1)

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
COUNT_MAX = 5

while True:
    re, frame = cap.read()

    # frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findCirclesGrid(gray, (cols, rows), flags=cv2.CALIB_CB_ASYMMETRIC_GRID)

    # success find chessboard
    if ret:
        objpoints.append(objp)

        imgpoints.append(corners)

        frame = cv2.drawChessboardCorners(frame, (cols, rows), corners, ret)
        print(corners)
        print(count)

        count += 1

    cv2.imshow('image', frame)
    sleep(0.5)

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
