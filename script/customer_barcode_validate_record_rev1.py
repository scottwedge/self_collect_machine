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
from std_msgs.msg import Int32

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg
import csv

class CustomerBarcodeRecord_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('CustomerBarcodeRecord_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# TODO: Un-comment for troubleshoot
		rospy.logwarn("Create an output for CUSTOMER Scanned QR-Code")

		self.rospack = rospkg.RosPack()
		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])
		self.outputDir = os.path.join(self.p, "csv")

		self.csv_filename = self.outputDir + "/customer_barcode" + ".csv"
		self.csv = open(self.csv_filename, "a")
		self.found = set()

		self.csv_filename_store = "/home/khairulizwan/catkin_ws/src/self_collect_machine/csv/store_barcode.csv"

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String)

		# TODO
		# Publish to the scan_status topic
		#self.scanStatus_pub = rospy.Publisher("/scan_status", String, queue_size=10)

		# Publish to the boxID_activation topic
		self.boxID_activation_pub = rospy.Publisher("/boxID_activation", Int32, queue_size=10)

		self.customerRecord()

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
			rospy.loginfo("[INFO] CustomerBarcodeRecord_node [OFFLINE]...")

		finally:
			pass

	def customerRecord(self):
		# TODO
		# Initiate the topic
		self.boxID_active = Int32()

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getQR()
			self.getMode()

			if self.mode.data == "customer":

				# TODO: Un-comment for troubleshoot
				#rospy.loginfo(self.barcodeData)

				# if the barcode text is currently not in our CSV file, write
				# the timestamp + barcode to disk and update the set
				if self.qr.data not in self.found:
					self.csv.write("{},{}\n".format(datetime.datetime.now(), self.qr.data))
					self.csv.flush()
					self.found.add(self.qr.data)

					with open(self.csv_filename_store, 'rt') as f_obj:
						reader = csv.reader(f_obj, delimiter=',')
						# Iterates through the rows of your csv
						for row in reader:
							if str(self.qr.data) == row[1]:
								# TODO: Un-comment for troubleshoot
								rospy.loginfo("Package(s) in box no {}".format(row[3]))

								# Publishing
								self.boxID_active.data = int(row[3])
								self.boxID_activation_pub.publish(self.boxID_active)

				else:
					rospy.logerr("IN CUSTOMER Record!")

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
