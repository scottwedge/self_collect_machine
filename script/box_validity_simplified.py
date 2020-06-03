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

		self.boxState = boxStatus()

		# Initializing your ROS Node
		rospy.init_node('box_validity', anonymous=False)

		# Subscribe Int32 msg
		self.sensorState1_sub = rospy.Subscriber("/sensorState_1", Int32)

		# Subscribe Int32 msg
		self.sensorState2_sub = rospy.Subscriber("/sensorState_2", Int32)

		# Subscribe Int32 msg
		self.sensorState3_sub = rospy.Subscriber("/sensorState_3", Int32)

		# Publish boxStatus msg
		self.boxStatus_pub = rospy.Publisher("/box_available", boxStatus, queue_size=10)

		self.getBoxState()

	def getSensorState1(self):

		self.state1 = rospy.wait_for_message("/sensorState_1", Int32)

	def getSensorState2(self):

		self.state2 = rospy.wait_for_message("/sensorState_2", Int32)

	def getSensorState3(self):

		self.state3 = rospy.wait_for_message("/sensorState_3", Int32)

	def getBoxState(self):

		while not rospy.is_shutdown():
			self.getSensorState1()
			self.getSensorState2()
			self.getSensorState3()

			self.boxState.data = [self.state1.data, self.state2.data, self.state3.data]
			self.boxStatus_pub.publish(self.boxState)

def main(args):

	vn = BoxIDValidate_node()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("[INFO] BoxIDValidate_node [OFFLINE]...")

if __name__ == '__main__':
	main(sys.argv)
