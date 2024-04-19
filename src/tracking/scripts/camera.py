#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)
        self.cap = cv2.VideoCapture(0)  # 0 is the device index of the camera (default camera)

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_message.header.frame_id = "camera_frame"  # Set the frame_id
            self.image_pub.publish(image_message)

if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=True)
    camera_node = CameraNode()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        camera_node.publish_image()
        rate.sleep()