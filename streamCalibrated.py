import cv2
from calibration import calibrated

cam_mtx = calibrated.getCameraMatrix()

cap = cv2.VideoCapture(1)
count = 0

while True:
    ret, frame = cap.read()
    # frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    result = calibrated.calibration(frame, cam_mtx)
    cv2.imshow('frame', result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()