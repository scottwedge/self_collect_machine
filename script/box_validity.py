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
from self_collect_machine.msg import boxStatus

import csv
import datetime
import cv2

class BoxIDValidate_node:
	def __init__(self):
		# Initializing your ROS Node
		rospy.init_node('BoxIDValidate_node', anonymous=True)

		rospy.on_shutdown(self.shutdown)

		# Subscribe to the sensorState_1 topic
		self.sensorState1_sub = rospy.Subscriber("/sensorState_1", 
			Int32, self.callbackSensorState1)

		# Subscribe to the sensorState_2 topic
		self.sensorState2_sub = rospy.Subscriber("/sensorState_2", 
			Int32, self.callbackSensorState2)

		# Subscribe to the sensorState_3 topic
		self.sensorState3_sub = rospy.Subscriber("/sensorState_3", 
			Int32, self.callbackSensorState3)

		# Publish to the scanned_barcode topic
		self.boxStatus_pub = rospy.Publisher("/box_available", 
			boxStatus, queue_size=1)

	def callbackSensorState1(self, data):
		self.boxID_1 = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.boxID_1)

	def callbackSensorState2(self, data):
		self.boxID_2 = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.boxID_2)

	def callbackSensorState3(self, data):
		self.boxID_3 = data.data

		# TODO: Un-comment for troubleshoot
		#rospy.loginfo(self.boxID_3)

		# Publishing
		self.scanBox = boxStatus()
		self.scanBox.data = [self.boxID_1, self.boxID_2, self.boxID_3]

		self.boxStatus_pub.publish(self.scanBox)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] BoxIDValidate_node [OFFLINE]...")

		finally:
			pass

def main(args):
	vn = BoxIDValidate_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BoxIDValidate_node [OFFLINE]...")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] BoxIDValidate_node [ONLINE]...")
	main(sys.argv)
