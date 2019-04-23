from utils.fs_utils import get_all_filenames
import os
from camera import checkerboard
import cv2

os.chdir('raw_rs/stream/session_calibration2018-10-22[23_28_43]/')
depth_images_path = get_all_filenames('', '*depth.png')

# print(depth_images_path)

color_images_path = get_all_filenames('', '*[!depth].png')

# print(color_images_path)

samples = 64

# import random

# sampled_paths = []

# i = 0
# while i < samples:
#     path = color_images_path[random.randrange(0, len(color_images_path) - 1)]
#     sampled_paths.append(path)
#     i += 1


image_paths = []
# images = [cv2.imread(img) for img in color_images_path]

mtx = checkerboard(color_images_path)

import numpy as np

print(np.matrix(mtx))
print(np.matrix(mtx).I)