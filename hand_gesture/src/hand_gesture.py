#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import cv2
#import time
#import argparse
#import imutils
import mediapipe as mp      # https://google.github.io/mediapipe/solutions/hands.html

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from collections import deque
from std_msgs.msg import String, Int16
from utils import CvFpsCalc
from gestures import *

def Hand_gesture():
    rospy.init_node("hand_gesture_node", anonymous=True)

    pub_hand = rospy.Publisher('/hand_gesture/image', Image, queue_size = 10)
    pub_number  = rospy.Publisher('/hand_gesture/number', Int16, queue_size = 10)
    
    bridge = CvBridge()

    # FPS Measurement
    cv_fps_calc = CvFpsCalc(buffer_len=10)

    # Argument parsing
    WRITE_CONTROL = False
    history_number = 0
    history_model = 0

    gesture_detector = GestureRecognition(min_detection_confidence=0.7, min_tracking_confidence=0.5)
    gesture_buffer = GestureBuffer(buffer_len=5)
    global gseture_id

    mode = 0
    number = -1

    cap = cv2.VideoCapture('http://192.168.0.122:8090/?action=stream')
    rate = rospy.Rate(1000) # 1000hz
    print("Start\n")

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        fps = cv_fps_calc.get()

        frame = cv2.flip(frame, 1)

        cam_info = CameraInfo()
        cam_info.width = 1280
        cam_info.height = 720
        cam_info.K = [3829.436319, 0.000000, 794.572896, 0.000000, 920.751044, 388.229605, 0.0, 0.0, 1.0]
        cam_info.D = [-0.331206, 0.072907, -0.016254, -0.025161, 0.000000]
        cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.P = [635.284485, 0.000000, 805.929614, 0.000000, 635.284485, 0.000000, 805.929614, 0.000000, 0.0, 0.0, 1.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        rate.sleep()

        key = cv2.waitKey(1) & 0xff
        if key == ord('n'):
            mode = 1
            WRITE_CONTROL = True

        if WRITE_CONTROL:
            number = -1
            if 48 <= key <= 57:  # 0 ~ 9
                number = key - 48
            elif 97 <= key <= 122:  # a ~ z
                number = key - 97 + 10

        debug_image, gesture_id = gesture_detector.recognize(frame, number, mode)
        gesture_buffer.add_gesture(gesture_id)

        debug_image = gesture_detector.draw_info(debug_image, fps, mode, number)
        history_id = -1

        if 0 <= gesture_id <= 9 and history_number >= 100:
            if 0 <= gesture_id <= 9 and history_model <= 50:
                history_model += 1
                #print(history_model)
            elif 0 <= gesture_id <= 9 and history_model >= 50:
                pub_number.publish(gesture_id)
                history_number = 0
                history_model = 0
                print('published\n')

        else:
            history_number += 1
            history_model = 0
            #print(history_number)

        cv2.imshow('Gesture Recognition', debug_image)
        pub_hand.publish(bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        cv2.waitKey(3)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        Hand_gesture()
    except rospy.ROSInterruptException:
        pass