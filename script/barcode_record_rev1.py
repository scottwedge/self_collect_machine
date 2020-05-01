#!/usr/bin/env python

################################################################################
## {Description}: Record the QR/Bar Code
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

class BarcodeRecord_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('BarcodeRecord_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# initialize the output directory path and create the output
		# directory
		rospy.logwarn("Create an output folder")

		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])
#		self.outputDir = os.path.join(self.p, datetime.datetime.now().strftime("%Y-%m-%d-%H%M"))
		self.outputDir = os.path.join(self.p, "csv")
		#os.makedirs(self.outputDir)

		#self.csv_filename = str(datetime.datetime.today()) + ".csv"
		self.csv_filename = self.outputDir + "/barcode" + ".csv"
		self.csv = open(self.csv_filename, "w")
		self.found = set()

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# TODO
		# Publish to the scan_status topic
		#self.scanStatus_pub = rospy.Publisher("/scan_status", String, queue_size=10)

		self.recordQR()

	# Get scanned barcode
	def getQR(self):
		# Wait for the topic
		self.qr = rospy.wait_for_message("/scanned_barcode", String)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] BarcodeRecord_node [OFFLINE]...")

		finally:
			pass

	def recordQR(self):
		# TODO
		# Initiate the topic
		#self.scanStatus = String()

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getQR()

			# if the barcode text is currently not in our CSV file, write
			# the timestamp + barcode to disk and update the set
			if self.qr.data not in self.found:
				self.csv.write("{},{}\n".format(datetime.datetime.now(), self.qr.data))
				self.csv.flush()

				self.found.add(self.qr.data)
				rospy.logwarn("Record IN")

			else:
				rospy.logerr("IN Record!")

def main(args):
	vn = BarcodeRecord_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BarcodeRecord_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] BarcodeRecord_node [ONLINE]...")
	main(sys.argv)
