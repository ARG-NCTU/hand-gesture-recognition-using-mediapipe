#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# https://maker.pro/raspberry-pi/projects/raspberry-pi-webcam-robot
# https://osoyoo.com/2020/04/30/install-a-web-camera-on-raspberry-pi/


import rospy
import numpy as np
import cv2
import mediapipe as mp      # https://google.github.io/mediapipe/solutions/hands.html

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from utils import CvFpsCalc

def image():
    bridge = CvBridge()
    rospy.init_node("seadrone_image_node", anonymous=True)

    mp_drawing = mp.solutions.drawing_utils
    mp_hands = mp.solutions.hands

    # For webcam input:
    hands = mp_hands.Hands(static_image_mode=True, max_num_hands=2, min_detection_confidence = 0.7, min_tracking_confidence = 0.7)

    # FPS Measurement
    cv_fps_calc = CvFpsCalc(buffer_len=5)

    pub_hand = rospy.Publisher('/mediapipe/image', Image, queue_size = 10)

    cap = cv2.VideoCapture('http://192.168.0.122:8090/?action=stream')
    rate = rospy.Rate(1000) # 1000hz
    print("Start\n")

    while not rospy.is_shutdown():
        fps = cv_fps_calc.get()

        ret, frame = cap.read()
        cam_info = CameraInfo()
        cam_info.width = 1280
        cam_info.height = 720
        cam_info.K = [3829.436319, 0.000000, 794.572896, 0.000000, 920.751044, 388.229605, 0.0, 0.0, 1.0]
        cam_info.D = [-0.331206, 0.072907, -0.016254, -0.025161, 0.000000]
        cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.P = [635.284485, 0.000000, 805.929614, 0.000000, 635.284485, 0.000000, 805.929614, 0.000000, 0.0, 0.0, 1.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        rate.sleep()

        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
    
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # Battery status and image rendering
        cv2.putText(image, "fps:{}".format(fps), (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        pub_hand.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
        cv2.imshow('hand_pose_seadrone', image)
        cv2.waitKey(3)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        image()
    except rospy.ROSInterruptException:
        pass