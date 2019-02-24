import numpy as np
import cv2
import pickle
import pprint

try:
    with open('calibration/camera_matrix', 'rb') as file:
        cam_mtx = pickle.load(file)
        pprint.pprint(cam_mtx)
except FileNotFoundError:
    print('Camera matrix not fonud. Please run `calibration/realtimeCalibrate.py` before.')

cap = cv2.VideoCapture(1)
count = 0

while True:
    ret, frame = cap.read()
    # frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx['mtx'], cam_mtx['dist'], (w,h), 1, (w,h))

    mapx, mapy = cv2.initUndistortRectifyMap(cam_mtx['mtx'], cam_mtx['dist'], None, newcameramtx, (w,h), 5)
    dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imshow('frame', dst)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()