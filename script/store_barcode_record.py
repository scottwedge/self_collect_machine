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
from self_collect_machine.msg import boxStatus

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg
import numpy as np

class StoreBarcodeRecord_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('StoreBarcodeRecord_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# open the output CSV file for writing and initialize the set of
		# barcodes found thus far
		# TODO:

		# initialize the output directory path and create the output
		# directory
		rospy.logwarn("Create an output folder")

		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])
#		self.outputDir = os.path.join(self.p, datetime.datetime.now().strftime("%Y-%m-%d-%H%M"))
		self.outputDir = os.path.join(self.p, "csv")
		#os.makedirs(self.outputDir)

		#self.csv_filename = str(datetime.datetime.today()) + ".csv"
		self.csv_filename = self.outputDir + "/store_barcode" + ".csv"
		self.csv = open(self.csv_filename, "w")
		self.found = set()

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String, 
				self.callbackScannedBar)

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String, 
				self.callbackScanMode)

		# Subscribe to the box_available topic
		self.boxStatus_sub = rospy.Subscriber("/box_available", boxStatus, 
				self.callbackBoxState)

		# Publish to the scan_status topic
		self.scanStatus_pub = rospy.Publisher("/scan_status", String, 
			queue_size=1)

	def callbackScanMode(self, data):
		self.scanMode = data.data

		#rospy.loginfo(self.scanMode)
		if self.scanMode == "store":
			#rospy.loginfo(self.barcodeData)
			if len(self.boxID) > 0:
				# if the barcode text is currently not in our CSV file, write
				# the timestamp + barcode to disk and update the set
				if self.barcodeData not in self.found:
					self.csv.write("{}, {}, {}\n".format(datetime.datetime.now(),
						self.barcodeData, self.boxID[0]))
					# TODO: Un-comment for troubleshoot
					rospy.logwarn("Saved as: {}, {}, {}\n".format(datetime.datetime.now(),
						self.barcodeData, self.boxID[0]))
					self.csv.flush()
					self.found.add(self.barcodeData)

				else:
					# Publishing
					self.scanStatus = String()
					self.scanStatus.data = "Scanned!"

					self.scanStatus_pub.publish(self.scanStatus)
			else:
				rospy.logerr("No Empty Box Available")

	def callbackScannedBar(self, data):
		self.barcodeData = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.barcodeData)

	def callbackBoxState(self, data):
		self.boxStatus = data.data

		self.boxID = np.array(self.boxStatus)
		self.boxID =  np.where(self.boxID == 0)[0]

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.boxStatus)
		#rospy.loginfo(self.boxID)
		#rospy.loginfo(len(self.boxID))

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] StoreBarcodeRecord_node [OFFLINE]...")

		finally:
			pass

def main(args):
	vn = StoreBarcodeRecord_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] StoreBarcodeRecord_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] StoreBarcodeRecord_node [ONLINE]...")
	main(sys.argv)
