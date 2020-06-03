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

		self.rospack = rospkg.RosPack()
		self.scanStatus = String()

		# Initializing your ROS Node
		rospy.init_node('store_barcode_record', anonymous=False)

		# initialize the output directory path and create the output
		# directory
		rospy.logwarn("Create an output folder")

		self.p = os.path.sep.join([self.rospack.get_path('self_collect_machine')])

		self.outputDir = os.path.join(self.p, "csv")
		self.csv_filename = self.outputDir + "/store_barcode" + ".csv"
		self.csv = open(self.csv_filename, "a")
		self.found = set()

		self.outputQRDir = os.path.join(self.p, "qr_code")

		# Subscribe String msg
		self.scannedBar_sub = rospy.Subscriber("/scanned_barcode", String)

		# Subscribe String msg
		self.scanMode_sub = rospy.Subscriber("/scan_mode", String)

		# Subscribe boxStatus msg
		self.boxStatus_sub = rospy.Subscriber("/box_available", boxStatus)

		# TODO
		# Publish String msg
		self.scanStatus_pub = rospy.Publisher("/scan_status", String, queue_size=10)

		self.storeRecord()

	def getQR(self):

		self.qr = rospy.wait_for_message("/scanned_barcode", String)

	def getMode(self):

		self.mode = rospy.wait_for_message("/scan_mode", String)

	def getBox(self):

		self.box = rospy.wait_for_message("/box_available", boxStatus)

	def storeRecord(self):

		while not rospy.is_shutdown():
			self.getQR()
			self.getMode()
			self.getBox()

			self.boxID = np.array(self.box.data)
			self.boxID = np.where(self.boxID == 1)[0]

			if self.mode.data == "store":
				if len(self.boxID) > 0:

					# if the barcode text is currently not in our CSV file, write
					# the timestamp + barcode to disk and update the set
					if self.qr.data not in self.found:
						# TODO: Publish a data to print on MAX2719

						self.csv.write("{},{},{}\n".format(datetime.datetime.now(), self.qr.data, self.boxID[0]))
						# TODO: Un-comment for troubleshoot
						rospy.logwarn("Saved as: {},{},{}\n".format(datetime.datetime.now(), self.qr.data, self.boxID[0]))
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
						self.scanStatus = "Scanned!"
						self.scanStatus_pub.publish(self.scanStatus)

				else:
					self.scanStatus = "No Empty Box Available"
					self.scanStatus_pub.publish(self.scanStatus)

def main(args):
	vn = StoreBarcodeRecord_node()

	try:
		rospy.spin()
	except KeyboardInterrupt as e:
		print(e)

if __name__ == '__main__':
	main(sys.argv)
