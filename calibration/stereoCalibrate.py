import cv2
import numpy as np
import sys
from time import sleep
import pickle
import pprint

args = sys.argv
cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

cols = 6
rows = 8

objp = np.zeros((rows*cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)

objpoints = []
imgpoints1 = []
imgpoints2 = []

result1 = {}
result2 = {}

count = 0
MAX = 20
if len(args) == 2:
    MAX = int(args[1])

def getCorners(img):
    global cols, rows
    flg = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_FAST_CHECK
    ret, corners = cv2.findChessboardCorners(img, (cols, rows), flags=flg)
    return ret, corners

def getCalibrationDict(objp, imgp, size):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objp, imgp, size, None,None)
    result = {}
    result['ret'] = ret
    result['mtx'] = mtx
    result['dist'] = dist
    result['rvecs'] = rvecs
    result['tvecs'] = tvecs
    return result

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    ret1, corners1 = getCorners(frame1)
    ret2, corners2 = getCorners(frame2)

    if ret1 and ret2:
        objpoints.append(objp)
        imgpoints1.append(corners1)
        imgpoints2.append(corners2)

        frame1 = cv2.drawChessboardCorners(frame1, (cols, rows), corners1, ret)
        frame2 = cv2.drawChessboardCorners(frame2, (cols, rows), corners2, ret)

        print(count)
        count += 1

    cv2.imshow('image_1', frame1)
    cv2.imshow('image_2', frame2)
    sleep(0.4)

    if count > MAX:
        result1 = getCalibrationDict(objpoints, imgpoints1, frame1.shape[::-1])
        result2 = getCalibrationDict(objpoints, imgpoints2, frame2.shape[::-1])

        pprint(result1)
        pprint(result2)
        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ret, frame = cap1.read()

ret, mtx1, dist1, mtx2, dist2, r, t, e, f = 
        cv2.stereoCalibrate(objpoints, imgpoints1, imgpoints2, frame.shape[::-1],
        result1['mtx'], result1['dist'], result2['mtx'], result2['dist'])

print(mtx1, mtx2)

cap.release()
cv2.destroyAllWindows()