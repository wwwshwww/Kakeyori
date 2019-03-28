import cv2
import numpy as np
import math
from calibration import calibrated
from coordinate import intersection

STEREO_DIST = 650
MM_PER_PIX = 0.00281

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

interpolation = cv2.INTER_NEAREST

s_mtx = calibrated.getStereoMatrix()
rect = calibrated.getStereoRect()
maps = calibrated.getStereoMap()

tmpret, tmpframe = cap1.read()
h, w = tmpframe.shape[:2]

sk = s_mtx['K1']
sdis = sk[0,0]
print(sdis)

print(maps)

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    frame1 = cv2.remap(frame1, maps['map1_l'], maps['map2_l'], interpolation)
    frame2 = cv2.remap(frame2, maps['map1_r'], maps['map2_r'], interpolation)

    corners1, ids1, rejectedImgPoints1 = aruco.detectMarkers(frame1, dictionary)
    corners2, ids2, rejectedImgPoints2 = aruco.detectMarkers(frame2, dictionary)

    if len(corners1) == 1 and len(corners2) == 1:
        # frame1 = aruco.drawDetectedMarkers(frame1, corners1, ids1)
        # frame2 = aruco.drawDetectedMarkers(frame2, corners2, ids2)

        inter1 = intersection.getQuadIntersection(corners1[0][0][:])
        inter2 = intersection.getQuadIntersection(corners2[0][0][:])

        cv2.circle(frame1, inter1, 20, (255, 255, 0), -1)
        cv2.circle(frame2, inter2, 20, (255, 255, 0), -1)

        tmp = math.sqrt((inter1[0] - inter2[0]) ** 2 + (inter1[1] - inter2[1]) ** 2)
        # d = (X vec of T) * (a11 of K1) / disparity
        d = (STEREO_DIST * sdis) / tmp
        print(d * MM_PER_PIX)

        px = (inter1[0] + inter2[0]) / 2. - w / 2.
        py = (inter1[1] + inter2[1]) / 2. - h / 2.

        rx = px * d * MM_PER_PIX ** 2
        ry = py * d * MM_PER_PIX ** 2

        print(rx, ry)

    stacked = np.hstack((frame1[:,:], frame2[:,:]))
    cv2.imshow('views', stacked)
    cv2.moveWindow('views', 0, 0)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
