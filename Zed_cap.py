#!/usr/bin/env python3

import cv2, time, sys
import glob
import numpy as np

cap = cv2.VideoCapture(0)

while(True):
# Capture frame by frame
    ret, frame = cap.read()

# partition video
    cv2.imshow("frame", frame)

