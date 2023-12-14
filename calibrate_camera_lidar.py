#!/usr/bin/env python3

# Python 2/3 compatibility
from __future__ import print_function

# Built-in modules
import os
import sys
import time
import threading
import multiprocessing

# External modules
import cv2
import numpy as np
import matplotlib.cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ROS modules
PKG = 'lidar_camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rosbag
import rospy
import tf2_ros
import ros_numpy
import image_geometry
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_matrix
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

# Global variables
OUSTER_LIDAR = False
PAUSE = False
FIRST_TIME = True
KEY_LOCK = threading.Lock()
TF_BUFFER = None
TF_LISTENER = None
CV_BRIDGE = CvBridge()
CAMERA_MODEL = image_geometry.PinholeCameraModel()

# Global paths
PKG_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CALIB_PATH = 'calibration_data/lidar_camera_calibration'