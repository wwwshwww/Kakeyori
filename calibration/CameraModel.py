import cv2
import numpy as np

# use chessboard when calibration

class WideCamera:
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_chess_flags = cv2.CALIB_CB_ADAPTIVE_THRESH \
                    + cv2.CALIB_CB_NORMALIZE_IMAGE \
                    + cv2.CALIB_CB_FILTER_QUADS \
                    + cv2.CALIB_CB_FAST_CHECK
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC \
                    + cv2.fisheye.CALIB_CHECK_COND \
                    + cv2.fisheye.CALIB_FIX_SKEW
    
    def __init__(self, device_index, chess_size):
        self.device_index = device_index
        self.cols = chess_size[0]
        self.rows = chess_size[1]
        self.cap = cv2.VideoCapture(self.device_index)
        self.objp = np.zeros((1, self.rows*self.cols, 3), np.float32)
        self.objp[0,:,:2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.objpoints = []
        self.imgpoints = []
        self.size = self.getSize()

    def __del__(self):
        self.cap.release()
        print('goodbye!')

    def getCapture(self):
        return self.cap

    def getFrame(self):
        return self.frame

    def read(self):
        self.available, self.frame = self.cap.read()

    def getGray(self):
        return cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

    def getSize(self):
        self.read()
        return self.getGray().shape[::-1]

    def findChess(self):
        self.found, self.corners = cv2.findChessboardCorners(self.frame, (self.cols, self.rows), flags=self.find_chess_flags)

    def getCorners(self):
        return self.corners

    def isFoundChess(self):
        return self.found

    def appendPoints(self):
        self.objpoints.append(self.objp)
        cv2.cornerSubPix(self.getGray(), self.corners, (11,11), (-1,-1), self.subpix_criteria)
        self.imgpoints.append(self.corners)

    def drawChess(self):
        frame = cv2.drawChessboardCorners(self.frame, (self.cols, self.rows), self.corners, self.found)
        return frame

    def calibrate(self):
        n = len(self.objpoints)
        rvecs = [np.zeros((1,1,3), dtype=np.float64) for i in range(n)]
        tvecs = [np.zeros((1,1,3), dtype=np.float64) for i in range(n)]
        ret, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
            self.objpoints,
            self.imgpoints,
            self.size,
            self.K,
            self.D,
            rvecs,
            tvecs,
            self.calibration_flags,
            (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
        print("K = np.array(" + str(k.tolist()) + ")")
        print("D = np.array(" + str(d.tolist()) + ")")

        return ret, k, d, rvecs, tvecs
