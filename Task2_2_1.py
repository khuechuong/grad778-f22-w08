#!/usr/bin/env python3

import numpy as np
import cv2
import matplotlib.pyplot as plt 
import glob


# Defnie the dimensions of checkerboard
dim =  (6, 9)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Initialization
points_3D = []          # Set of 3D corners 
points_2D = []          # Set of 2D corners in pixel coordinate
object_3d = np.zeros((1, dim[0] * dim[1], 3), np.float32)  # Set of chessboard corners

# Extract the index of chessboard corners due to the chessboard dimension
object_3d[0, :, :2] = np.mgrid[0:dim[0],
                               0:dim[1]].T.reshape(-1, 2)          # like (0, 0, 0), (1, 0, 0), ....

# Create a list of string of filepath to image file
# images = sorted(glob.glob('*.png'))
images = sorted(glob.glob('*.png'))

for filename in images:
    # Read each image
    img  = cv2.imread(filename)

    # Convert image into gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(
                    gray, dim, cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret == True:
        points_3D.append(object_3d)

        # Refining pixel coordinates for 2D pixels
        corners_2D = cv2.cornerSubPix(gray, corners, (9, 9),
                                     (-1, -1), criteria)
        points_2D.append(corners_2D)

        # Draw and display the corners 
        frame = cv2.drawChessboardCorners(img, (6, 9), corners_2D, ret)
        cv2.imwrite("chessboard {0}.png".format(filename), frame)


    ## Perform camera calibration by matching the 3D points and its 2D 
#  Corresponding pixel coordinates
ret, intric_matrix, distortion, r_matrix, t_vecs = cv2.calibrateCamera(points_3D,
        points_2D, gray.shape[::-1], None, None)

# Displaying output
print("Camera's intrinsic matrix:")
print(intric_matrix)

print("\n Distortion coefficient:")
print(distortion)

print("\n Rotation matrix:")
print(r_matrix)

print("\n Translation vectors:")
print(t_vecs)

