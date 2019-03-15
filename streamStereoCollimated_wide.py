import cv2
import numpy as np
from calibration import calibrated

cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

interpolation = cv2.INTER_CUBIC

rect = calibrated.getStereoRect()
maps = calibrated.getStereoMap()

print(maps)

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    re_frame1 = cv2.remap(frame2, maps['map1_l'], maps['map2_l'], interpolation)
    re_frame2 = cv2.remap(frame1, maps['map1_r'], maps['map2_r'], interpolation)

    # re_frame1 = cv2.blur(re_frame1,(5,5))
    # re_frame2 = cv2.blur(re_frame2,(5,5))

    stacked = np.hstack((re_frame1[:,:], re_frame2[:,:]))
    cv2.imshow('collimated views', stacked)

    gray1 = cv2.cvtColor(re_frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(re_frame2, cv2.COLOR_BGR2GRAY)

    # 画像のヒストグラム平坦化・平滑化
    gray1 = cv2.GaussianBlur(cv2.equalizeHist(gray1),(5,5), 0)
    gray2 = cv2.GaussianBlur(cv2.equalizeHist(gray2),(5,5), 0)

    stereo = cv2.StereoBM_create(80,5)

    # block_size = 27
    # min_disp = 0
    # num_disp = 112
    # stereo = cv2.StereoSGBM_create(
    #     minDisparity = min_disp,           # 視差の下限
    #     numDisparities = num_disp,        # 最大の上限
    #     blockSize = block_size,
    #     mode = True
    # )


    disparity = stereo.compute(gray1, gray2).astype(np.float32) / 700.0
    # disparity = cv2.GaussianBlur(disparity,(21,21),0)
    # disparity = cv2.medianBlur(disparity, 5)
    for i in range(3):
        # disparity = cv2.blur(disparity, (1+(i*2),1+(i*2)))
        disparity = cv2.GaussianBlur(disparity,(7, 7),0)
    cv2.imshow('disparity', disparity)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()
