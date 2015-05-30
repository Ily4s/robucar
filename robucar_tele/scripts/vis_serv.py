#!/usr/bin/env python

__author__ = "Ilyas M. Abbas (ily4s.abbas@gmail.com)"

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):
		cv2.namedWindow("Image window", 1)
		rospy.Subscriber("/camera/image_raw",Image,self.callback)
		self.bridge = CvBridge()

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print e

		print str(cv_image.shape)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(3)

if __name__ == '__main__':
	ic = image_converter()
	rospy.init_node('image_subscriber', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()