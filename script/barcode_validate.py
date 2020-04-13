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

class BarcodeValidate_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('BarcodeValidate_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# TODO:
		# open the output CSV file for writing and initialize the set of
		# barcodes found thus far
		self.csv_filename = "/home/khairulizwan/catkin_ws/src/self_collect_machine/csv/barcode.csv"
#		self.csv = open("/home/khairulizwan/catkin_ws/src/self_collect_machine/csv/" + self.csv_filename, "w")
#		self.found = set()

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String, self.validate_callback)

	def validate_callback(self,data):
		# Convert the raw image to OpenCV format
		rospy.loginfo("I heard %s", data.data)

		a = data.data

		with open(self.csv_filename) as f_obj:
			reader = csv.reader(f_obj, delimiter=',')
			#Iterates through the rows of your csv
			for line in reader:
				#line here refers to a row in the csv
				print(line)
				#If the string you want to search is in the row
				if a in line:
					print("String found in {} row of csv".format(line[1]))

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

			# if the barcode text is currently not in our CSV file, write
			# the timestamp + barcode to disk and update the set
			if self.barcodeData not in self.found:
				self.csv.write("{},{}\n".format(datetime.datetime.now(),
					self.barcodeData))
				self.csv.flush()
				self.found.add(self.barcodeData)

				# Publishing
				self.scanCode = String()
				self.scanCode.data = self.barcodeData

				self.scannedBar_pub.publish(self.scanCode)

			else:
				cv2.putText(self.cv_image, "Scanned!", 
					(100, 100), cv2.FONT_HERSHEY_DUPLEX, 1, 
					(0, 255, 0), 1, cv2.LINE_AA, False)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] BarcodeValidate_node [OFFLINE]...")

		finally:
			pass

def main(args):
	vn = BarcodeValidate_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BarcodeValidate_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] BarcodeValidate_node [ONLINE]...")
	main(sys.argv)
