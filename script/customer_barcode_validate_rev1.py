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
from std_msgs.msg import Int32

import csv
import datetime
import cv2

class CustomerBarcodeValidate_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('CustomerBarcodeValidate_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# open the output CSV file for writing and initialize the set of
		# barcodes found thus far
		self.csv_filename = "/home/khairulizwan/catkin_ws/src/self_collect_machine/csv/store_barcode.csv"

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String)

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# Publish to the boxID_activation topic
		self.boxID_activation_pub = rospy.Publisher("/boxID_activation", Int32, queue_size=10)

		self.customerValidate()

	# Get scanned barcode
	def getQR(self):
		# Wait for the topic
		self.qr = rospy.wait_for_message("/scanned_barcode", String)

	# Get scanned barcode
	def getMode(self):
		# Wait for the topic
		self.mode = rospy.wait_for_message("/scan_mode", String)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] CustomerBarcodeValidate_node [OFFLINE]...")

		finally:
			pass

	def callbackScanMode(self, data):
		self.scanMode = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.scanMode)

	def customerValidate(self):
		# TODO
		# Initiate the topic
		self.boxID_active = Int32()

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getQR()
			self.getMode()

			if self.mode.data == "customer":
				with open(self.csv_filename) as f_obj:
					reader = csv.reader(f_obj, delimiter=',')
					# Iterates through the rows of your csv
					for line in reader:
						# TODO: Un-comment for troubleshoot
						# line here refers to a row in the csv
						#print(line)
						# If the string you want to search is in the row
						if self.qr.data in line:
							# TODO: Un-comment for troubleshoot
							rospy.loginfo("Package(s) in box no {}".format(line[3]))

							# Publishing
							self.boxID_active.data = line[3]
							self.boxID_activation_pub.publish(self.boxID_active)

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
