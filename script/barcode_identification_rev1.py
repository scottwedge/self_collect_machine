#!/usr/bin/env python

################################################################################
## {Description}: Identify the QR/Bar Code -- CustomerQR or StoreQR
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

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg

import re

class BarcodeIdentity_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('BarcodeIdentity_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Subscribe to the scanned_barcode topic
		# self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String, self.callback)
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# Publish to the scan_status topic
		self.typeQR_pub = rospy.Publisher("/scan_mode", String, queue_size=10)

		self.getMode()

	# Get scanned barcode
	def getQR(self):
		# Wait for the topic
		self.qr = rospy.wait_for_message("/scanned_barcode", String)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] BarcodeIdentity_node [OFFLINE]...")

		finally:
			pass

	def getMode(self):
		# Initiate the topic
		self.typeQR = String()

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getQR()

			# Identify the barcode data
			# Current approach:
			# StoreQR: OrderID & CustomerEmail
			# CustomerQR: OrderID
			lst = re.findall('\S+@\S+', self.qr.data)

			if len(lst) == 1:
				# TODO: Un-comment for troubleshoot
				#rospy.loginfo("Store Mode")

				self.typeQR.data = "store"

			elif len(lst) == 0:
				# TODO: Un-comment for troubleshoot
				#rospy.loginfo("Customer Mode")

				# TODO:
				self.typeQR.data = "customer"

			self.typeQR_pub.publish(self.typeQR)

def main(args):
	vn = BarcodeIdentity_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BarcodeIdentity_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] BarcodeIdentity_node [ONLINE]...")
	main(sys.argv)
