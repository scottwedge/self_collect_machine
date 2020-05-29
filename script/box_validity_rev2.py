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
#		self.sensorState1_sub = rospy.Subscriber("/sensorState_1", Int32)
		self.sensorState1_sub = rospy.Subscriber("/switchState_1", Int32)

		# Subscribe to the sensorState_2 topic
#		self.sensorState2_sub = rospy.Subscriber("/sensorState_2", Int32)
		self.sensorState1_sub = rospy.Subscriber("/switchState_2", Int32)

		# Subscribe to the sensorState_3 topic
#		self.sensorState3_sub = rospy.Subscriber("/sensorState_3", Int32)
		self.sensorState3_sub = rospy.Subscriber("/switchState_3", Int32)

		# TODO: may add more
		# Subscribe to the sensorState_N topic
		#self.sensorStateN_sub = rospy.Subscriber("/sensorState_N", Int32)

		# Publish to the scanned_barcode topic
		self.boxStatus_pub = rospy.Publisher("/box_available", boxStatus, queue_size=10)

		self.getBoxState()

	def getSensorState1(self):
		# Wait for the topic
		self.state1 = rospy.wait_for_message("/switchState_1", Int32)

	def getSensorState2(self):
		# Wait for the topic
		self.state2 = rospy.wait_for_message("/switchState_2", Int32)

	def getSensorState3(self):
		# Wait for the topic
		self.state3 = rospy.wait_for_message("/switchState_3", Int32)

	# TODO: may add more
	#def getSensorStateN(self):
		# Wait for the topic
		#self.stateN = rospy.wait_for_message("/sensorState_N", String)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] BoxIDValidate_node [OFFLINE]...")

		finally:
			pass

	def getBoxState(self):
		# Initiate the topic
		self.boxState = boxStatus()

		while not rospy.is_shutdown():
			# Get the scan-ed data
			self.getSensorState1()
			self.getSensorState2()
			self.getSensorState3()
			# TODO: May add more here

			self.boxState.data = [self.state1.data, self.state2.data, self.state3.data]
			self.boxStatus_pub.publish(self.boxState)

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
