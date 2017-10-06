#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class WebcamNode(object):
  def __init__(self):
    rospy.init_node('webcam_node')
    self.pub = rospy.Publisher('/image_raw', Image, queue_size=1)
    self.cap = cv2.VideoCapture(0)
    self.bridge = CvBridge()
    print "Initialized webcam node!"
    self.run()

  def run(self):
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
      ret, frame = self.cap.read()

      msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
      msg.header.stamp = rospy.get_rostime()
      self.pub.publish(msg)

      rate.sleep()

    self.cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
  try:
    node = WebcamNode()
  except rospy.ROSInterruptException:
    pass
      