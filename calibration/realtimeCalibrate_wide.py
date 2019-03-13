import cv2
# import matplotlib.pyplot as plt
import numpy as np
import sys
from time import sleep
import pickle
import pprint
import calibrated

from CameraModel import WideCamera

args = sys.argv

COLS = 6
ROWS = 7

if len(args) == 2:
    COUNT_MAX = int(args[1])
else:
    COUNT_MAX = 40

count = 0

result = {}
cam1 = WideCamera(1, (COLS,ROWS))

while True:
    cam1.read()
    frame = cam1.getFrame()
    cam1.findChess()
    # success find chessboard
    if cam1.isFoundChess():
        cam1.appendPoints()
        frame = cam1.drawChess()

        print(cam1.getCorners())
        print(count)
        count += 1

    cv2.imshow('image', frame)
    sleep(0.3)

    if count > COUNT_MAX:
        ret, K, D, rvecs, tvecs = cam1.calibrate()
        result['ret'] = ret
        result['mtx'] = K
        result['dist'] = D
        result['rvecs'] = rvecs
        result['tvecs'] = tvecs

        pprint.pprint(result)
        calibrated.outputter('camera_matrix', result)

        break

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

del cam1
cv2.destroyAllWindows()
