#!/usr/bin/env python3

################################################################################
## {Description}: Validate the QR/Bar Code (for USB type camera)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

import sys
import rospy

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from self_collect_machine.msg import boxStatus

import RPi.GPIO as GPIO
from time import sleep, strftime
from datetime import datetime

from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.led_matrix.device import max7219
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, LCD_FONT

class BoxIDDisplay_node:
	def __init__(self):
		self.serial = spi(port=0, device=0, gpio=noop())
		self.device = max7219(self.serial, width=32, height=8, block_orientation=-90)
		self.device.contrast(5)
		self.virtual = viewport(self.device, width=32, height=16)

		self.sensor_val = 0

		# Connect sensor topic
		self.sensor_topic = "/boxNumber"
#		self.sensor_sub = rospy.Subscriber(self.sensor_topic, Int32, self.callback)
		self.sensor_sub = rospy.Subscriber(self.sensor_topic, Int32)

		# Allow up to one second to connection
		rospy.sleep(1)

	def getSensor(self):
		# Wait for the topic
		self.sensor = rospy.wait_for_message(self.sensor_topic, Int32)

	def update_display(self):
		self.getSensor()

		if self.sensor.data != self.sensor_val:
			show_message(self.device, 'Box: {}'.format(self.sensor.data), fill="white", font=proportional(LCD_FONT), scroll_delay=0.08)

			with canvas(self.virtual) as draw:
				text(draw, (1, 1), "Box:{}".format(self.sensor.data), fill="white", font=proportional(LCD_FONT))

			self.sensor_val = self.sensor.data
		else:
			pass

if __name__ == '__main__':

	# Initialize
	rospy.init_node('BoxIDDisplay_node', anonymous=False)
	led = BoxIDDisplay_node()

	# Take a photo
	while not rospy.is_shutdown():
		led.update_display()

	# Sleep to give the last log messages time to be sent
	rospy.sleep(1)
