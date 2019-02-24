import glob
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys

args = sys.argv
cap = cv2.VideoCapture(1)

# setting chessboard
if len(args) == 3:
    cols = args[1]
    rows = args[2]
else:
    cols = 5
    rows = 5

while True:
    ret, frame = cap.read()

    # frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(frame, (cols, rows))

    # success find chessboard
    if ret:
        print('corners shape', corners.shape)
        
