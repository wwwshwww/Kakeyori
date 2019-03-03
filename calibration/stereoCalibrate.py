import cv2
import numpy as np
import sys
from time import sleep
import pickle
import pprint

args = sys.argv
cap1 = cv2.VideoCapture(1)
cap2 = cv2.VideoCapture(2)

cols = 6
rows = 8

num = 20
if len(args) == 2:
    num = int(args[1])

objp1 = 