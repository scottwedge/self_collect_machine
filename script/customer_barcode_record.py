#!/usr/bin/env python

################################################################################
## {Description}: Record the QR/Bar Code on CustomerDatabase
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

class CustomerBarcodeRecord_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('CustomerBarcodeRecord_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# TODO: Un-comment for troubleshoot
		rospy.logwarn("Create an output folder")

		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])
		self.outputDir = os.path.join(self.p, "csv")

		self.csv_filename = self.outputDir + "/customer_barcode" + ".csv"
		self.csv = open(self.csv_filename, "w")
		self.found = set()

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String, self.callbackScannedBar)

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String, self.callbackScanMode)

		# Publish to the scan_status topic
		self.scanStatus_pub = rospy.Publisher("/scan_status", String, queue_size=1)

	def callbackScanMode(self, data):
		self.scanMode = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.scanMode)

		if self.scanMode == "customer":
			#rospy.loginfo(self.barcodeData)

			# if the barcode text is currently not in our CSV file, write
			# the timestamp + barcode to disk and update the set
			if self.barcodeData not in self.found:
				self.csv.write("{},{}\n".format(datetime.datetime.now(),
					self.barcodeData))
				self.csv.flush()
				self.found.add(self.barcodeData)

			else:
				# Publishing
				self.scanStatus = String()
				self.scanStatus.data = "Scanned!"

				self.scanStatus_pub.publish(self.scanStatus)

	def callbackScannedBar(self, data):
		self.barcodeData = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.barcodeData)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] CustomerBarcodeRecord_node [OFFLINE]...")

		finally:
			pass

def main(args):
	vn = CustomerBarcodeRecord_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] CustomerBarcodeRecord_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] CustomerBarcodeRecord_node [ONLINE]...")
	main(sys.argv)
