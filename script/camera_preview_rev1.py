#!/usr/bin/env python

################################################################################
## {Description}: Preview an Image from USB type camera
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class CameraPreview_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('CameraPreview_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Camera Preview"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/cv_camera/image_raw", Image)

		# Subscribe to the camera info topic
		self.imgInfo_sub = rospy.Subscriber("/cv_camera/camera_info", CameraInfo)

		self.cvtImage()

	def getImage(self):
		# Wait for the topic
		self.image = rospy.wait_for_message("/cv_camera/image_raw", Image)

	# Get the width and height of the image
	def getCameraInfo(self):
		# Wait for the topic
		self.camerainfo = rospy.wait_for_message("/cv_camera/camera_info", CameraInfo)

	# Overlay some text onto the image display
	def textInfo(self):
		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		# Get the scan-ed data
		self.getCameraInfo()

		cv2.putText(self.cv_image, "Sample", (10, self.camerainfo.height-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA, False)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.camerainfo.width, self.camerainfo.height), (self.camerainfo.width-100, self.camerainfo.height-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA, False)

	# Refresh the image on the screen
	def dispImage(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	# Shutdown
	def shutdown(self):
		try:
			rospy.logwarn("CameraPreview_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	# Convert the raw image to OpenCV format
	def cvtImage(self):
		while not rospy.is_shutdown():
			try:
				# Get the scan-ed data
				self.getImage()

				# Convert the raw image to OpenCV format """
				self.cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")

				# TODO:
				#self.cv_image = imutils.rotate(self.cv_image, angle=180)
				self.cv_image_copy = self.cv_image.copy()

				# Overlay some text onto the image display
				self.textInfo()

				# Refresh the image on the screen
				self.dispImage()

			except CvBridgeError as e:
				print(e)

def main(args):
	vn = CameraPreview_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("CameraPreview_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("CameraPreview_node [ONLINE]...")
	main(sys.argv)
