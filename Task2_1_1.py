#!/usr/bin/env python3
import cv2
import os

right_camera = True
left_camear  = False

def camera_capture(device_num, basename, ext='png', delay=1, window_name='frame'):
    cap = cv2.VideoCapture(device_num)

    if not cap.isOpened():
        return


    n = 0
    while True:
        ret, frame = cap.read()

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(delay) & 0xFF
            
        if key == ord('s'):
            cv2.imwrite('{}_{}.{}'.format(basename, n, ext), frame)
            n += 1
        elif key == ord('q'):
            break

    cv2.destroyWindow(window_name)


camera_capture(0, 'camera_capture')
