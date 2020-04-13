#!/usr/bin/env python

################################################################################
## {Description}: Code (Bar/QR) Recognition using USB type camera
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
		self.imgRaw_sub = rospy.Subscriber("/cv_camera/image_raw", 
				Image, self.callback)

		# Subscribe to the camera info topic
		self.imgInfo_sub = rospy.Subscriber("/cv_camera/camera_info", 
				CameraInfo, self.getCameraInfo)

		# Subscribe to the scan_status topic
		self.imgInfo_sub = rospy.Subscriber("/scan_status", 
				String, self.callback_status)

		# Publish to the scanned_barcode topic
		self.scannedBar_pub = rospy.Publisher("/scanned_barcode", String, 
			queue_size=1)

	def callback(self,data):
		# Convert the raw image to OpenCV format
		self.cvtImage(data)

		# Overlay some text onto the image display
		self.textInfo()

		self.barcodeScan()

		# Refresh the image on the screen
		self.displayImg()

	# 
	def callback_status(self, data):
		self.scanStatus = data.data

		rospy.loginfo(self.scanStatus)

		# TODO:
#		# draw the barcode data and barcode type on the image
#		cv2.putText(self.cv_image, "{}".format(self.scanStatus), (100, 100), 
#			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

	# Get the width and height of the image
	def getCameraInfo(self, msg):
		self.image_width = msg.width
		self.image_height = msg.height

	# Convert the raw image to OpenCV format
	def cvtImage(self, data):
		try:
			# Convert the raw image to OpenCV format """
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# TODO:
			#self.cv_image = imutils.rotate(self.cv_image, angle=180)
			self.cv_image_copy = self.cv_image.copy()

		except CvBridgeError as e:
			print(e)

	def barcodeScan(self):
		# find the barcodes in the frame and decode each of the barcodes
		self.barcodes = pyzbar.decode(self.cv_image)

		# loop over the detected barcodes
		for self.barcode in self.barcodes:
			# extract the bounding box location of the barcode and 
			# draw the bounding box surrounding the barcode on the 
			# image
			(self.x, self.y, self.w, self.h) = self.barcode.rect
			cv2.rectangle(self.cv_image, (self.x, self.y), 
				(self.x + self.w, self.y + self.h), 
				(0, 0, 255), 2)

			# the barcode data is a bytes object so if we want to 
			# draw it on our output image we need to convert it to 
			# a string first
			self.barcodeData = self.barcode.data.decode("utf-8")
			self.barcodeType = self.barcode.type

			# draw the barcode data and barcode type on the image
			cv2.putText(self.cv_image, "{} ({})".format(
				self.barcodeData, self.barcodeType), (self.x, 
				self.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
				(0, 0, 255), 2)

			# Publishing
			self.scanCode = String()
			self.scanCode.data = self.barcodeData

			self.scannedBar_pub.publish(self.scanCode)

	# Overlay some text onto the image display
	def textInfo(self):
		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 0.5
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		cv2.putText(self.cv_image, "Sample", (10, self.image_height-10), 
			cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1, 
			cv2.LINE_AA, False)
		cv2.putText(self.cv_image, "(%d, %d)" % (self.image_width, 
			self.image_height), (self.image_width-100, 
			self.image_height-10), cv2.FONT_HERSHEY_DUPLEX, 0.5, 
			(255, 255, 255), 1, cv2.LINE_AA, False)

	# Refresh the image on the screen
	def displayImg(self):
		cv2.imshow(self.cv_window_name, self.cv_image)
		cv2.waitKey(1)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("BarcodeRecognition_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

def main(args):
	vn = BarcodeRecognition_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("BarcodeRecognition_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("BarcodeRecognition_node [ONLINE]...")
	main(sys.argv)
