#!/usr/bin/env python

################################################################################
## {Description}: Record the QR/Bar Code on CustomerDatabase and Sent Email
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
from self_collect_machine.msg import boxStatus

from pyzbar import pyzbar
import datetime
import time
import os
import rospkg
import numpy as np

import pyqrcode

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders
import os.path

class StoreBarcodeRecord_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('StoreBarcodeRecord_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# initialize the output directory path and create the output
		# directory
		rospy.logwarn("Create an output folder")

		self.rospack = rospkg.RosPack()

		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])

		self.outputDir = os.path.join(self.p, "csv")
		self.csv_filename = self.outputDir + "/store_barcode" + ".csv"
		self.csv = open(self.csv_filename, "a")
		self.found = set()

		self.outputQRDir = os.path.join(self.p, "qr_code")

		# Subscribe to the scanned_barcode topic
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# Subscribe to the scan_mode topic
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String)

		# Subscribe to the box_available topic
		self.boxStatus_sub = rospy.Subscriber("/box_available", boxStatus)

		# Publish to the boxNumber topic
		self.scanBox_pub = rospy.Publisher("/boxNumber", Int32, queue_size=10)

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
			rospy.loginfo("[INFO] StoreBarcodeRecord_node [OFFLINE]...")

		finally:
			pass

	def storeRecord(self):
		# TODO
		# Initiate the topic
		self.scanBox = Int32

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getQR()
			self.getMode()
			self.getBox()

			self.boxID = np.array(self.box.data)

			# TODO:
			self.boxID = np.where(self.boxID == 1)[0]

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
						# TODO: Publish a data to print on MAX2719
						self.scanBox.data = self.boxID[0]
						self.scanBox_pub.publish(self.scanBox)

						self.csv.write("{},{},{}\n".format(datetime.datetime.now(), 
							self.qr.data,, self.boxID[0]))

						# TODO: Un-comment for troubleshoot
						rospy.logwarn("Saved as: {},{},{}\n".format(datetime.datetime.now(), 								self.qr.data, self.boxID[0]))
						self.csv.flush()
						self.found.add(self.qr.data)

						# Generate QR Code
						self.custQR = self.qr.data.rsplit(',', 1)[0]
						self.urlcustomer = pyqrcode.create(self.custQR)

						self.customerQR = self.outputQRDir + "/" + self.custQR + ".png"

						self.urlcustomer.png(self.customerQR, scale=8)

						# Email QR Code
						email = 'wansnap@gmail.com' # Your email
						password = 'Kh@irulizwan1984' # Your email account password
						send_to_emails = [self.qr.data.rsplit(',', 1)[1], 'shafikahdarwis@gmail.com'] # List of email addresses
						subject = "Order ID " + self.custQR # The subject line
						message = "Please proceed to our Self Collect Machine, and show the Code" # The message in the email
						file_location = self.customerQR

						# Create the attachment file (only do it once)
						filename = os.path.basename(file_location)
						attachment = open(file_location, "rb")
						part = MIMEBase('application', 'octet-stream')
						part.set_payload(attachment.read())
						encoders.encode_base64(part)
						part.add_header('Content-Disposition', "attachment; filename= %s" % filename)

						# Connect and login to the email server
						server = smtplib.SMTP('smtp.gmail.com', 587)
						server.starttls()
						server.login(email, password)

						# Loop over each email to send to
						for send_to_email in send_to_emails:
							# Setup MIMEMultipart for each email address (if we don't do this, the emails will concat on each email sent)
							msg = MIMEMultipart()
							msg['From'] = email
							msg['To'] = send_to_email
							msg['Subject'] = subject

							# Attach the message to the MIMEMultipart object
							msg.attach(MIMEText(message, 'plain'))
							# Attach the attachment file
							msg.attach(part)

							# Send the email to this specific email address
							server.sendmail(email, send_to_email, msg.as_string()) 

						# Quit the email server when everything is done
						rospy.logwarn("Detail has been emailed")
						server.quit()

					else:
						rospy.logerr("IN Record!")

				else:
					rospy.logerr("No Empty Box Available")


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
