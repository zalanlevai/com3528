#!/usr/bin/env python3

import cv2
import numpy as np

# Based on https://github.com/MiRo-projects/miro2_examples/blob/master/common/camera_configs.py

left_camera_matrix = np.array([
    [401.0750, 0, 0],
    [0, 396.4960, 0],
    [322.1910, 178.9279, 1]
]).T
left_distortion = np.array([-0.05783, 0.00739, 0.0000, 0.00000, 0.190000])

right_camera_matrix = np.array([
    [401.1001, 0, 0],
    [0, 396.447, 0],
    [312.6818, 183.3658, 1]
]).T
right_distortion = np.array([-0.0216, 0.27, 0.0000, 0.00000, -0.19000])

R = np.array([
    [0.9057, 0.00196, -0.4246],
    [-0.0182, 0.9976, -0.0106],
    [0.4244, -0.01709, 0.9052]
])
T = np.array([-1.1088, -0.0716, -0.2885])

size = (640, 360)

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
    left_camera_matrix, left_distortion,
    right_camera_matrix, right_distortion,
    size, R, T
)

left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
