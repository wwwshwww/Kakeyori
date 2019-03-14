import cv2
import numpy as np
import sys
from time import sleep
from CameraModel import WideCamera
from pprint import pprint
import pickle
import calibrated

args = sys.argv

COLS = 6
ROWS = 8

if len(args) == 2:
    COUNT_MAX = int(args[1])
else:
    COUNT_MAX = 40

count = 0

cam1 = WideCamera(1, (COLS, ROWS))
cam2 = WideCamera(2, (COLS, ROWS))

def allClose():
    global cam1, cam2
    del cam1, cam2
    cv2.destroyAllWindows()

while True:
    cam1.read()
    cam2.read()
    frame1 = cam1.getFrame()
    frame2 = cam2.getFrame()
    cam1.findChess()
    cam2.findChess()

    if cam1.isFoundChess() and cam2.isFoundChess():
        cam1.appendPoints()
        cam2.appendPoints()
        frame1 = cam1.drawCorners()
        frame2 = cam2.drawCorners()
        print(count)
        count += 1

    stacked_img = np.hstack((frame1[:,:], frame2[:,:]))
    
    cv2.imshow('view', stacked_img)
    cv2.moveWindow('view', 0, 200)

    sleep(0.3)

    if count > COUNT_MAX:
        ret1, K1, D1, rvecs1, tvecs1 = cam1.calibrate()
        ret2, K2, D2, rvecs2, tvecs2 = cam2.calibrate()

        break
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        allClose()
        exit()

objp = cam1.getObjectPoints()

n = len(cam1.getObjectPoints())

R = np.zeros((1, 1, 3), dtype=np.float64)
T = np.zeros((1, 1, 3), dtype=np.float64)

objp = np.zeros((COLS*ROWS, 1, 3) , np.float64)
objp[:,0, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2)

imgp1 = cam1.getImagePoints()
imgp2 = cam2.getImagePoints()

objpoints = np.array([objp]*len(imgp1), dtype=np.float64)
imgpoints_left = np.asarray(imgp1, dtype=np.float64)
imgpoints_right = np.asarray(imgp2, dtype=np.float64)

objpoints = np.reshape(objpoints, (n, 1, COLS*ROWS, 3))
imgpoints_left = np.reshape(imgpoints_left, (n, 1, COLS*ROWS, 2))
imgpoints_right = np.reshape(imgpoints_right, (n, 1, COLS*ROWS, 2))

size = cam1.getSize()

K1 = cam1.getK()
D1 = cam1.getD()
K2 = cam2.getK()
D2 = cam2.getD()

ret, K1, D1, K2, D2, R, T = cv2.fisheye.stereoCalibrate(
    objpoints,
    imgpoints_left,
    imgpoints_right,
    K1,
    D1,
    K2,
    D2,
    size,
    R,
    T,
    WideCamera.calibration_flags,
    WideCamera.calibration_criteria
)

print('ret:\n', ret, '\nK1:\n', K1,'\nD1:\n', D1,'\nK2:\n', K2,'\nD2:\n', D2,'\nR:\n', R,'\nT:\n', T)

mtx = {}

mtx['ret'] = ret
mtx['K1'] = K1
mtx['D1'] = D1
mtx['K2'] = K2
mtx['D2'] = D2
mtx['R'] = R
mtx['T'] = T

calibrated.outputter('stereo_matrices', mtx)

R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
    K1,
    D1,
    K2,
    D2,
    cam1.getSize(),
    R,
    T,
    0
)
pprint((R1, R2, P1, P2, Q))

stereo_rect = {}
stereo_rect['R1'] = R1
stereo_rect['R2'] = R2
stereo_rect['P1'] = P1
stereo_rect['P2'] = P2
stereo_rect['Q'] = Q

calibrated.outputter('stereo_rect', stereo_rect)

m1type = cv2.CV_32FC1
map1_l, map2_l = cv2.fisheye.initUndistortRectifyMap(K1, D1, R1, P1, cam1.getSize(), m1type)
map1_r, map2_r = cv2.fisheye.initUndistortRectifyMap(K2, D2, R2, P2, cam2.getSize(), m1type)

print(map1_l, map2_l)
print(map2_l, map2_r)

stereo_map = {}
stereo_map['map1_l'] = map1_l
stereo_map['map1_r'] = map1_r
stereo_map['map2_l'] = map2_l
stereo_map['map2_r'] = map2_r

calibrated.outputter('stereo_map', stereo_map)

allClose()

    