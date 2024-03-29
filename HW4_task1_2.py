#!/usr/bin/env python3

import numpy as np
import cv2
import glob
import argparse 
import math        
import rospy
from std_msgs.msg    import Float64
from geometry_msgs.msg import Vector3

t_vecs_aruco = 0


ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}
#############################################
## Function to estimate pose of the marker ##
#############################################
def pose_estimation(frame, intric_matrix, distortion):
    # global t_vecs_aruco
    # global theta
    d = 0
    # Detect corners, ids( vectors of identifiers), rejected_image_points
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    # Detect the aruco marker
    detected_markers, th, id = aruco_show(corners, ids, rejected, frame)

    if len(corners) > 0:

        # R_matrix and T_vecs different from camera frame to ArUCo
        r_matrix_aruco, t_vecs_aruco,  _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, intric_matrix, distortion)

        for i in range(0, ids.size):
            # Draw axis for the ArUCo markers
            cv2.drawFrameAxes(frame, intric_matrix, distortion, r_matrix_aruco[i], t_vecs_aruco[i], 0.1)
        
        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(frame, corners)

        # Calculate the distance sepration
        d =  math.sqrt(t_vecs_aruco[0][0][0]*t_vecs_aruco[0][0][0] + 
                     t_vecs_aruco[0][0][1]*t_vecs_aruco[0][0][1])

        # print("distance_sepration", d)

    return frame, d, th, id

##################################################
## Function to display the boundary of ArUCo tag##
##################################################
def aruco_show(corners, ids, rejected, image):
    theta = 0
    m_id = 0
    if len(corners) > 0:
        # flatten the ArUCo IDs list:
        ids = ids.flatten()

        # Extract the frame size
        h, w, _ = image.shape

        for (marker_corner, marker_ID) in zip (corners, ids):
            # Extract the marker corners (top-left, top-right, 
            # bottom-right, bottom-left in order)
            corners  = marker_corner.reshape((4, 2))
            
            # Extract each corner point
            (top_left, top_right, bottom_right, bottom_left) = corners

            # Convert to integers
            top_left     = (int(top_left[0]), int(top_left[1]))
            top_right    = (int(top_right[0]), int(top_right[1]))
            bottom_left  = (int(bottom_left[0]), int(bottom_left[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))

            # Draw the boundary lines by connecting each corner point
            cv2.line(image, top_left, top_right, (0, 255, 0), 2)
            cv2.line(image, top_left, bottom_left, (0, 255, 0), 2)
            cv2.line(image, bottom_right, top_right, (0, 255, 0), 2)
            cv2.line(image, bottom_right, bottom_left, (0, 255, 0), 2)
            
            # Compute the camera center in x-y coordinate of the ArUCo marker
            cX = int((top_left[0] + bottom_right[0]) / 2.0)
            cY = int((top_left[1] + bottom_right[1]) / 2.0)

            # Draw corners and center points
            cv2.circle(image, (top_left[0], top_left[1]), 4, (255, 0, 0), -1)
            cv2.circle(image, (top_right[0], top_right[1]), 4, (255, 0, 0), -1) 
            cv2.circle(image, (bottom_right[0], bottom_right[1]), 4, (255, 0, 0), -1) 
            cv2.circle(image, (bottom_left[0], bottom_left[1]), 4, (255, 0, 0), -1) 
            cv2.circle(image, (cX, cY), 4, (0, 0, 255),  -1)
            

            # Draw the ArUCo marker ID on the image
            cv2.putText(image, str(marker_ID), (top_left[0], top_left[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5
                        , (0, 255, 0), 2)

            theta = (cX - 316.94)*60/640

            print("ArUco marker ID: {}".format(marker_ID))
            m_id = marker_ID
            # print("Alignment_error", (cX - 316.94)*60/640)
    return image, theta, m_id
#########################
## Publish the message ##
#########################
def navigation():
    # define the actions the publisher will make
    # pub_r = rospy.Publisher("/aligment_error", Float64, queue_size=10)
    # pub_t = rospy.Publisher("/distance_separation", Float64, queue_size=10)

    

    # rate = rospy.Rate(5)

    pass
    # keep a buffer based on the rate defined earlier 
    # rate.sleep() 

######################### Calibration #################################
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
images = (glob.glob('*.png'))


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
        corners_2D = cv2.cornerSubPix(gray, corners, (11, 11),
                                     (-1, -1), criteria)

        points_2D.append(corners_2D)

#  Corresponding pixel coordinates
ret, intric_matrix, distortion, r_matrix, t_vecs = cv2.calibrateCamera(points_3D,
    points_2D, gray.shape[::-1], None, None)
##################################################################################

#################### Aruco detection and Pose estimation #########################
# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--camera", required=True, help="Set to True if using webcam")
ap.add_argument("-v", "--video", help="Path to the video file")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
args = vars(ap.parse_args())

video = cv2.VideoCapture(0)

# Load the ArUCo dictionary, ArUCo parameters, and detect the markers
aruco_dict   = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
aruco_params = cv2.aruco.DetectorParameters_create()

pub_ = rospy.Publisher("/mix", Vector3, queue_size=10)
    # initalize the publishing nobe
rospy.init_node("aligment_node", anonymous=True)

while video.isOpened():
    # Capture the video frame by frame
    ret, frame = video.read()

    # Estimate the ArUCO pose:
    pose, a, b, c = pose_estimation(frame, intric_matrix, distortion)

    # cv2.imshow("Estimated Pose", pose)

    # The 'q' button is set as the quitting button
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    vector = Vector3()
    # if theta != 0:
    vector.x = float(a)
    vector.y = b
    vector.z = c
    # else:
    #     vector.x = 0
    #     vector.y = 0

    # pub_r.publish(theta)
    # pub_t.publish(d)

    pub_.publish(vector)


video.release()
cv2.destroyAllWindows()

