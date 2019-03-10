import cv2
import numpy as np
import sys
from time import sleep
from CameraModel import WideCamera

args = sys.argv

COLS = 6
ROWS = 7

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
        frame1 = cam1.drawChess()
        frame2 = cam2.drawChess()
        print(count)
        count += 1

    stacked_img = np.hstack((frame1[:,:], frame2[:,:]))
    
    cv2.imshow('view', stacked_img)
    sleep(0.3)

    if count > COUNT_MAX:
        ret1, K1, D1, rvecs1, tvecs1 = cam1.calibrate()
        ret2, K2, D2, rvecs2, tvecs2 = cam2.calibrate()

        pprint.pprint((K1, K2))
        break
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        allClose()
        exit()

ret, K1, D1, K2, D2, R, T = cv2.fisheye.stereoCalibrate(
    cam1.getObjectPoints(),
    cam1.getImagePoints(),
    cam2.getImagePoints(),
    cam1.getK(),
    cam1.getD(),
    cam2.getK(),
    cam2.getD(),
    cam1.getSize()
)
print(ret, K1, D1, K2, D2, R, T)

allClose()

    