#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# to start the camera: roslaunch openni2_launch openni2.launch
bridge = CvBridge()

def callback(data):
  cv_image = bridge.imgmsg_to_cv2(data)
  # do whatever with the image (for now i'll show)
  cv2.imshow("window", cv_image)
  cv2.waitKey(3)

def listener():
  # init node and subscribe to the camera image
  rospy.Subscriber("camera/depth/image", Image, callback)
  rospy.init_node("camera_depth_img", anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == "__main__":
  listener()
