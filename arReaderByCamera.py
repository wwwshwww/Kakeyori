import cv2

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

def arReader():
    cap = cv2.VideoCapture(0) # start VideoCapture

    while True:
        ret, frame = cap.read() 

        frame = cv2.resize(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)))

        corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary)
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # coordinates of marker
        font = cv2.FONT_HERSHEY_PLAIN
        index = 0;
        for c in corners:
          mid = ids[index]
          x = 0.0;
          y = 0.0;
          for xy in c:
            x += xy[0][0]
            y += xy[0][1]
          x = x / 4
          y = y / 4
          index += 1;

          print('[' + str(mid) + '] x: ' + str(x) + ' y: ' + str(y))
        #.putText(frame, corners, (frame.shape[1]/2, frame.shape[0]/2), font, 0.6, (255,255,0))

        cv2.imshow('Edited frame', frame)

        k = cv2.waitKey(1)
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

arReader()