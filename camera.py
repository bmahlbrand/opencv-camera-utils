import numpy as np
import cv2

from streamCamera import *

import time
import datetime

import glob

import random

def auto_canny(image, sigma=0.33):

    v = np.median(image)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    return edged

def checkerboard(images):

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)


    objp = objp.reshape(-1,1,3)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    # gray = cv2.cvtColor(imag,cv2.COLOR_BGR2GRAY)

    for fname in images:
        # print(fname)
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            # img = cv2.drawChessboardCorners(gray, (7,6), corners2,ret)
            # cv2.imshow('image', img)
            # cv2.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    tot_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        tot_error += error
        
    print()
    print("mean error: ", "{0:.2f}".format((tot_error / len(objpoints)) * 100), '%')
    print()
    print('recovered projection matrix:')
    print(mtx)
    return mtx

def checkerboardVideo(filename):

    video = cv2.VideoCapture(filename)
    total = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    frames = list(range(0, total))

    frame_samples = random.sample(frames, 50)
    
    print('frames in video: ', str(total))

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)


    objp = objp.reshape(-1,1,3)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    # gray = cv2.cvtColor(imag,cv2.COLOR_BGR2GRAY)

    for sample in frame_samples:
        video.set(cv2.CAP_PROP_POS_FRAMES, sample)
        
        ret, img = video.read()

        # cv2.imshow('frame', img)
        # cv2.waitKey(500)
        # img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(gray, (7,6), corners2,ret)
            cv2.imshow('image', img)
            cv2.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    tot_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        tot_error += error
        # print(tot_error)
        # print(error)

    print()
    print("mean error: ", "{0:.2f}".format((tot_error / len(objpoints)) * 100), '%')
    print()
    print('recovered projection matrix:')
    print(mtx)

# checkerboardVideo(videoDump + 'output2018-10-11[02_48_12].avi')

# images = glob.glob('data/checkerboard_samples/*.jpg')
# print(images)

ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d[%H_%M_%S]')

# checkerboard(images)

# i = input('calibration phase...')
# print()

# if i == ' ' or i == '':
#     cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
#     saveStream(0, videoDump + st + 'output_checkerboard' + '.avi')

# i = input('motion phase...')
# print()

# if i == ' ' or i == '':
#     cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
#     saveStream(0, videoDump + st + 'output' + '.avi')

# playVideo('output.avi')
# stream(0, checkerboard)
# stream(0)
saveStream(0, 'stream.avi')