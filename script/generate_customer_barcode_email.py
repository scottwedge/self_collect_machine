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

class EmailQRCode_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('EmailQRCode_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])
		self.outputDir = os.path.join(self.p, "csv")

		self.csv_filename_store = self.outputDir + "/store_barcode" + ".csv"

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String)

		# Subscribe to the box_available topic
		self.boxStatus_sub = rospy.Subscriber("/box_available", boxStatus)

		# TODO
		# Publish to the scan_status topic
		#self.scanStatus_pub = rospy.Publisher("/scan_status", String, queue_size=10)

		self.storeRecord()

	# Get scanned barcode
	def getQR(self):
		# Wait for the topic
		self.qr = rospy.wait_for_message("/scanned_barcode", String)

	# Get scanned barcode
	def getMode(self):
		# Wait for the topic
		self.mode = rospy.wait_for_message("/scan_mode", String)

	# Get scanned barcode
	def getBox(self):
		# Wait for the topic
		self.box = rospy.wait_for_message("/box_available", boxStatus)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] EmailQRCode_node [OFFLINE]...")

		finally:
			pass

	def storeRecord(self):
		# TODO
		# Initiate the topic
		#self.scanStatus = String()

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getQR()
			self.getMode()
			self.getBox()

			self.boxID = np.array(self.box.data)
			self.boxID = np.where(self.boxID == 0)[0]

			#rospy.loginfo(self.scanMode)
			if self.mode.data == "store":
				# TODO: Un-comment for troubleshoot
				#rospy.loginfo(self.qr.data)

				if len(self.boxID) > 0:
					# TODO: Un-comment for troubleshoot
					#rospy.logwarn("Empty Box Available")

					# if the barcode text is currently not in our CSV file, write
					# the timestamp + barcode to disk and update the set
					if self.qr.data not in self.found:
						self.csv.write("{},{},{}\n".format(datetime.datetime.now(), self.qr.data, self.boxID[0]))
						# TODO: Un-comment for troubleshoot
						rospy.logwarn("Saved as: {},{},{}\n".format(datetime.datetime.now(), self.qr.data, self.boxID[0]))
						self.csv.flush()
						self.found.add(self.qr.data)

					else:
						rospy.logerr("IN Record!")

				else:
					rospy.logerr("No Empty Box Available")


def main(args):
	vn = EmailQRCode_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] EmailQRCode_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] EmailQRCode_node [ONLINE]...")
	main(sys.argv)
