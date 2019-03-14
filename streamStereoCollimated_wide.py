import cv2
import numpy as np
from calibration import calibrated

cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

interpolation = cv2.INTER_NEAREST

rect = calibrated.getStereoRect()
maps = calibrated.getStereoMap()

print(maps)

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    re_frame1 = cv2.remap(frame2, maps['map1_l'], maps['map2_l'], interpolation)
    re_frame2 = cv2.remap(frame1, maps['map1_r'], maps['map2_r'], interpolation)

    stacked = np.hstack((re_frame1[:,:], re_frame2[:,:]))
    cv2.imshow('collimated views', stacked)

    # cv2.imshow('collimated view1', re_frame1)
    # cv2.imshow('collimated view2', re_frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
