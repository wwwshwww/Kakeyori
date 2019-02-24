import glob
import cv2
# import matplotlib.pyplot as plt
import numpy as np
import sys
from time import sleep

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

args = sys.argv
cap = cv2.VideoCapture(1)

# setting chessboard
if len(args) == 3:
    cols = int(args[1])
    rows = int(args[2])
else:
    cols = 5
    rows = 4

while True:
    re, frame = cap.read()

    frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners_temp = cv2.findCirclesGrid(gray, (cols, rows), cv2.CALIB_CB_ASYMMETRIC_GRID)

    cv2.imshow('Edited frame', gray)

    # success find chessboard
    if ret:
        corners = cv2.cornerSubPix(gray,corners_temp,(11,11),(-1,-1),criteria)
        print('corners shape: ', corners.shape)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    sleep(0.1)

cap.release()
cv2.destroyAllWindows()
