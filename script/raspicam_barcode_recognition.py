#!/usr/bin/env python

################################################################################
## {Description}: Code (Bar/QR) Recognition using Raspberry Pi Camera (raspicam)
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
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

from pyzbar import pyzbar
import datetime
import time

class BarcodeRecognition_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('BarcodeRecognition_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Give the OpenCV display window a name
		self.cv_window_name = "Barcode Recognition"

		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to the raw camera image topic
		self.imgRaw_sub = rospy.Subscriber("/raspicam_node_robot/image/compressed", CompressedImage)

		# Publish to the scanned_barcode topic
		self.scannedBar_pub = rospy.Publisher("/scanned_barcode", String, queue_size=10)

		self.cvtImage()

	def getImage(self):
		# Wait for the topic
		self.image = rospy.wait_for_message("/raspicam_node_robot/image/compressed", CompressedImage)

	# Get the width and height of the image
	def getCameraInfo(self):
		self.image_width = rospy.get_param("/raspicam_node_robot/width") 
		self.image_height = rospy.get_param("/raspicam_node_robot/height")

	# TODO
	# Overlay some text onto the image display
	def textInfo(self):
		self.getCameraInfo()

		cv2.putText(self.cv_image, "Sample", (10, self.image_height-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA, False)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.image_width, self.image_height), (self.image_width-100, self.image_height-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA, False)

	# Refresh the image on the screen
	def dispImage(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] Raspicam_Preview_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	def getBarcode(self):
		while not rospy.is_shutdown():
			try:
				# Get the scan-ed data
				self.getImage()

				# direct conversion to CV2 ####
				self.cv_image = np.fromstring(self.image.data, np.uint8)
				self.cv_image = cv2.imdecode(self.cv_image, cv2.IMREAD_COLOR)

				# OPTIONAL -- image-rotate """
				self.cv_image = imutils.rotate(self.cv_image, angle=-90)
				self.cv_image = cv2.flip(self.cv_image,1)

				# find the barcodes in the frame and decode each of the barcodes
				self.barcodes = pyzbar.decode(self.cv_image)

				# loop over the detected barcodes
				for self.barcode in self.barcodes:
					# TODO: Un-comment for troubleshoot
					# Get the scan-ed data
					#self.getScanStatus()

					# extract the bounding box location of the barcode and 
					# draw the bounding box surrounding the barcode on the 
					# image
					(self.x, self.y, self.w, self.h) = self.barcode.rect
					cv2.rectangle(self.cv_image, (self.x, self.y), (self.x + self.w, self.y + self.h), (0, 0, 255), 2)

					# the barcode data is a bytes object so if we want to 
					# draw it on our output image we need to convert it to 
					# a string first
					self.barcodeData = self.barcode.data.decode("utf-8")
					self.barcodeType = self.barcode.type

					# TODO: Un-comment for troubleshoot
					#rospy.loginfo(self.barcodeData)

					# TODO: Un-comment for troubleshoot
					# draw the barcode data and barcode type on the image
					cv2.putText(self.cv_image, "{} ({})".format(self.barcodeData, self.barcodeType), (self.x, self.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

					# Publishing
					self.scanCode = String()
					self.scanCode.data = self.barcodeData

					self.scannedBar_pub.publish(self.scanCode)

				# Overlay some text onto the image display
				self.textInfo()

				# Refresh the image on the screen
				self.dispImage()

			except CvBridgeError as e:
				print(e)


def main(args):
	vn = BarcodeRecognition_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BarcodeRecognition_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] BarcodeRecognition_node [ONLINE]...")
	main(sys.argv)
