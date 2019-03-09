import cv2
from calibration import calibrated

wide = calibrated.getCameraMatrix()

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    # frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))
    result = calibrated.getUndistortedImage_wide(frame, wide)
    cv2.imshow('result', result)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()