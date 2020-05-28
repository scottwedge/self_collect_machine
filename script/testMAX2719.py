#!/usr/bin/env python3

import RPi.GPIO as GPIO
from time import sleep, strftime
from datetime import datetime

from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.led_matrix.device import max7219
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, LCD_FONT

import random

serial = spi(port=0, device=0, gpio=noop())
device = max7219(serial, width=32, height=8, block_orientation=-90)
device.contrast(5)
virtual = viewport(device, width=32, height=16)
#show_message(device, 'Raspberry Pi MAX7219', fill="white", font=proportional(LCD_FONT), scroll_delay=0.08)

try:
	while True:
		show_message(device, 'S3L4M4T H4R1 R4Y4 1441H', fill="white", 
			font=proportional(LCD_FONT), scroll_delay=random.uniform(0.05, 0.08))
		device.contrast(random.randint(5, 255))

		#with canvas(virtual) as draw:
			#text(draw, (0, 1), "Idris", fill="white", font=proportional(CP437_FONT))
			#text(draw, (0, 1), datetime.now().strftime('%I:%M'), fill="white", font=proportional(CP437_FONT))

except KeyboardInterrupt:
	GPIO.cleanup()
