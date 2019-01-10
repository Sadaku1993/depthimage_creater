#!/usr/bin/env python2
# -*- coding:utf-8 -*-

import numpy as np
import cv2
import glob
import sys

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

objpoints = []
imgpoints = []

img = cv2.imread("0.jpg")
if img is None:
    print("Fail to Read")
    sys.exit(1)
    
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

if ret == True:
    objpoints.append(objp)

    corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    imgpoints.append(corners2)

    # Draw and display the corners
    img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)

    while True:
        cv2.imshow('img',img)
        cv2.waitKey(500)
        if cv2.waitKey(100) == 27:
            break
