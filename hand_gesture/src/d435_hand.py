#!/usr/bin/python3

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
from std_msgs.msg import String
from utils import CvFpsCalc

class Detection(object):
    def __init__(self):
        self.bridge = CvBridge()

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        # For webcam input:
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence = 0.7, min_tracking_confidence = 0.7)

        # FPS Measurement
        self.cv_fps_calc = CvFpsCalc(buffer_len=5)

        # ROS Publisher & Subscriber
        self.sub_img = message_filters.Subscriber('/seadrone/camera/color/image_raw', Image)
        self.pub_img = rospy.Publisher('/seadrone/camera/mediapipe/image_raw', Image, queue_size = 1)

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
        image = cv2.cvtColor(cv2.flip(cv_image, 1), cv2.COLOR_BGR2RGB)
    
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = self.hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                for id, lm in enumerate(hand_landmarks.landmark):
                    h, w, c = image.shape
                    cx, cy = int(lm.x * w), int(lm.y *h)
                    print(id, cx, cy)
                    if id == 4 or id == 8 or id == 12 or id == 16 or id == 20:
                        cv2.circle(image, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

                self.mp_drawing.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        # image size 640*480
        cv2.putText(image, "fps:{}".format(fps), (15, 470), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(image, "8UC3"))
        cv2.imshow('hand_pose_d435', image)
        cv2.waitKey(3)

    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")

if __name__ == "__main__":
    rospy.init_node("d435_hand_pose_node", anonymous=False)
    node = Detection()
    cv2.destroyAllWindows()
    rospy.on_shutdown(node.shutdown_cb)

    rospy.spin()