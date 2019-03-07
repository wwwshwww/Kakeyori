import cv2
import os

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

def arGenerator():
    fileName = "ar.png"
    generator = aruco.drawMarker(dictionary, 23, 100)
    cv2.imwrite(fileName, generator)

    img = cv2.imread(fileName)
    cv2.imshow('ArMaker',img)
    cv2.waitKey(0)
arGenerator()