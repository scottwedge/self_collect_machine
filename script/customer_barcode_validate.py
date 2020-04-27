#!/usr/bin/env python

################################################################################
## {Description}: Validate the QR/Bar Code (for USB type camera)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

from __future__ import print_function
from __future__ import division

import sys
import rospy

from std_msgs.msg import String

import csv
import datetime
import cv2

class CustomerBarcodeValidate_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('CustomerBarcodeValidate_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# TODO:
		# open the output CSV file for writing and initialize the set of
		# barcodes found thus far
		self.csv_filename = "/home/khairulizwan/catkin_ws/src/self_collect_machine/csv/store_barcode.csv"
#		self.csv = open("/home/khairulizwan/catkin_ws/src/self_collect_machine/csv/" + self.csv_filename, "w")
#		self.found = set()

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String, 
				self.callbackScanMode)

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", 
				String, self.validate_callback)

	# TODO:
	def callbackScanMode(self, data):
		self.scanMode = data.data

		#
		#rospy.loginfo(self.scanMode)

	def validate_callback(self,data):
		# 
		#rospy.loginfo("I heard %s", data.data)

		a = data.data

		if self.scanMode == "customer":
			with open(self.csv_filename) as f_obj:
				reader = csv.reader(f_obj, delimiter=',')
				#Iterates through the rows of your csv
				for line in reader:
#					#line here refers to a row in the csv
#					print(line)
					#If the string you want to search is in the row
					if a in line:
						rospy.loginfo("Package(s) in box no {}".format(line[3]))
#					elif a not in line:
#						rospy.logerr("Not in record!")

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] CustomerBarcodeValidate_node [OFFLINE]...")

		finally:
			pass

def main(args):
	vn = CustomerBarcodeValidate_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] CustomerBarcodeValidate_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] CustomerBarcodeValidate_node [ONLINE]...")
	main(sys.argv)
