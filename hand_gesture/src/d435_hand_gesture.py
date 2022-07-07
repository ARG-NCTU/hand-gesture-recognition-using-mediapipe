#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import os
import numpy as np
import cv2
import math
import time
import json
import mediapipe as mp      # https://google.github.io/mediapipe/solutions/hands.html

# ROS
import rospy
import roslib
import rospkg
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from collections import deque
from std_msgs.msg import String, Int16
from utils import CvFpsCalc
from gestures import *

class Hand_gesture(object):
    def __init__(self):
        self.bridge = CvBridge()

        # FPS Measurement
        self.cv_fps_calc = CvFpsCalc(buffer_len=5)

        # ROS Publisher & Subscriber
        self.sub_img = message_filters.Subscriber('/seadrone/camera/color/image_raw', Image)
        self.pub_hand = rospy.Publisher('/seadrone/camera/hand_gesture/image_raw', Image, queue_size = 10)
        self.pub_number  = rospy.Publisher('/hand_gesture/number', Int16, queue_size = 10)

        # Argument parsing
        self.WRITE_CONTROL = False
        self.history_number = 0
        self.history_model = 0

        self.gesture_detector = GestureRecognition(min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.gesture_buffer = GestureBuffer(buffer_len=5)
        global gesture_id

        self.mode = 0
        self.number = -1

        rospy.loginfo('Detection node ready.')

        ts = message_filters.TimeSynchronizer([self.sub_img], 1)
        ts.registerCallback(self.callback)

    def callback(self, rgb):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")
        except CvBridgeError as e:
            print(e)

        fps = self.cv_fps_calc.get()

        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        # image = cv2.cvtColor(cv2.flip(cv_image, 1), cv2.COLOR_BGR2RGB)
        image = cv2.flip(cv_image, 1)

        key = cv2.waitKey(1) & 0xff
        if key == ord('n'):
            self.mode = 1
            self.WRITE_CONTROL = True

        if self.WRITE_CONTROL:
            self.number = -1
            if 48 <= key <= 57:  # 0 ~ 9
                self.number = key - 48
            elif 97 <= key <= 122:  # a ~ z
                self.number = key - 97 + 10

        debug_image, gesture_id = self.gesture_detector.recognize(image, self.number, self.mode)
        self.gesture_buffer.add_gesture(gesture_id)

        debug_image = self.gesture_detector.draw_info(debug_image, fps, self.mode, self.number)
        history_id = -1

        if 0 <= gesture_id <= 9 and self.history_number >= 100:
            if 0 <= gesture_id <= 9 and self.history_model <= 50:
                self.history_model += 1
                #print(self.history_model)
            elif 0 <= gesture_id <= 9 and self.history_model >= 50:
                self.pub_number.publish(gesture_id)
                self.history_number = 0
                self.history_model = 0
                print('published\n')

        else:
            self.history_number += 1
            self.history_model = 0
            #print(self.history_number)

        cv2.imshow('Gesture Recognition', debug_image)
        self.pub_hand.publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))
        cv2.waitKey(3)

    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")

if __name__ == "__main__":
    rospy.init_node("d435_hand_gesture_node", anonymous=True)
    node = Hand_gesture()
    cv2.destroyAllWindows()
    rospy.on_shutdown(node.shutdown_cb)

    rospy.spin()