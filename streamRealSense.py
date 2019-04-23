import pyrealsense2 as rs

import numpy as np
import cv2

rawDumpFolder = 'raw_rs'
imageDump = rawDumpFolder + '/' + 'screenshots' + '/'
videoDump = rawDumpFolder + '/' + 'stream' + '/'

import time
import datetime

from utils.fs_utils import create_folder

import random

def saveStream(folder):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    print('...start session')
    # cap = cv2.VideoCapture(device)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # depthFileStream = cv2.VideoWriter(folder + 'depth.avi', fourcc, 20.0, (640,480))
    depthColorFileStream = cv2.VideoWriter(folder + 'depth_colormap.avi', fourcc, 20.0, (640,480))
    rgbFileStream = cv2.VideoWriter(folder + 'rgb.avi', fourcc, 20.0, (640,480))

    try:
        while True:
            
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.10), cv2.COLORMAP_JET)

            # depthFileStream.write(cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET))
            
            depthColorFileStream.write(depth_colormap)
            rgbFileStream.write(color_image)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            if cv2.waitKey(1) & 0xFF == ord('q'):
               break
    finally:

        # Stop streaming
        pipeline.stop()

def stream():

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    print('...start session')
    i = 0
    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            # cv2.waitKey(1)
            key = cv2.waitKey(1)
        
            if key & 0xFF == ord('s'):
                cv2.imwrite(imageDump + 'frame' + str(i) + '.png', color_image)
                cv2.imwrite(imageDump + 'frame' + str(i) + '_depth' + '.png', depth_image)
            elif key & 0xFF == ord('q'):
                break
            i += 1

    finally:

        # Stop streaming
        pipeline.stop()

def streamSaveSampled(folder):

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    print('...start session')
    i = 0
    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            # cv2.waitKey(1)

            if random.randrange(1, 100) < 10:
                cv2.imwrite(folder + 'frame' + str(i) + '.png', color_image)
                cv2.imwrite(folder + 'frame' + str(i) + '_depth' + '.png', depth_image)

            key = cv2.waitKey(1)
        
            # if key & 0xFF == ord('s'):
            if key & 0xFF == ord('q'):
                break
            i += 1

    finally:

        # Stop streaming
        pipeline.stop()

ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d[%H_%M_%S]')

dirName = 'session_calibration' + st

folderPath = videoDump + dirName + '/'
create_folder(folderPath)
streamSaveSampled(folderPath)

#######################################################################

print('general pose capture phase...')

dirName = 'session_' + st

folderPath = videoDump + dirName + '/'
create_folder(folderPath)
streamSaveSampled(folderPath)