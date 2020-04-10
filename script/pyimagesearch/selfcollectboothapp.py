# import the necessary packages
from __future__ import print_function
from PIL import Image
from PIL import ImageTk
import Tkinter as tki
import threading
import datetime
import imutils
import cv2
import os

# TODO:
from pyzbar import pyzbar
import time
import jdutil

class BarcodeBoothApp:
	def __init__(self, vs, outputPath, csv, found):
		# store the video stream object and output path, then initialize
		# the most recently read frame, thread for reading frames, and
		# the thread stop event
		self.vs = vs
		self.outputPath = outputPath
		self.frame = None
		self.thread = None
		self.stopEvent = None

		self.csv = csv
		self.found = found

		# initialize the root window and image panel
		self.root = tki.Tk()
		self.panel = None

		# TODO:
#		# create a button, that when pressed, will take the current
#		# frame and save it to file
#		btn = tki.Button(self.root, text="Snapshot!",
#			command=self.takeSnapshot)
#		btn.pack(side="bottom", fill="both", expand="yes", padx=10,
#			pady=10)

		# TODO:
		self.time1 = ''

		self.date_etc = tki.Label(self.root, font=('arial', 10, 'bold'), 
				fg='red', bg='black')
		self.date_etc.pack(side="bottom", fill="both", expand="yes", padx=0,
				pady=0)

		self.clock_lt = tki.Label(self.root, font=('arial', 30, 'bold'), 
				fg='red', bg='black')
		self.clock_lt.pack(side="bottom", fill="both", expand="yes", padx=0,
				pady=0)

		self.date_iso = tki.Label(self.root, font=('arial', 10, 'bold'), 
				fg='red',bg='black')
		self.date_iso.pack(side="bottom", fill="both", expand="yes", padx=0,
				pady=0)

#		self.clock_utc = tki.Label(self.root, font=('arial', 30, 'bold'),
#				fg='red')
#		self.clock_utc.pack(side="bottom", fill="both", expand="yes", padx=10,
#				pady=10)

		# start a thread that constantly pools the video sensor for
		# the most recently read frame
		self.stopEvent = threading.Event()
		self.thread = threading.Thread(target=self.videoLoop, args=())
		self.thread.start()

		# set a callback to handle when the window is closed
		self.root.wm_title("SELF COLLECT BOOTH")
		self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

		# TODO
		self.root.configure(background='black')

		self.tick()

	def videoLoop(self):
		# DISCLAIMER:
		# I'm not a GUI developer, nor do I even pretend to be. This
		# try/except statement is a pretty ugly hack to get around
		# a RunTime error that Tkinter throws due to threading
		try:
			# keep looping over frames until we are instructed to stop
			while not self.stopEvent.is_set():
				# grab the frame from the video stream and resize it to
				# have a maximum width of 300 pixels
				self.frame = self.vs.read()
				self.frame = imutils.resize(self.frame, width=300)

				# TODO:
				# find the barcodes in the frame and decode each of the barcodes
				self.barcodes = pyzbar.decode(self.frame)

				# loop over the detected barcodes
				for self.barcode in self.barcodes:
					# extract the bounding box location of the barcode and draw
					# the bounding box surrounding the barcode on the image
					(x, y, w, h) = self.barcode.rect
					cv2.rectangle(self.frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

					# the barcode data is a bytes object so if we want to draw it
					# on our output image we need to convert it to a string first
					self.barcodeData = self.barcode.data.decode("utf-8")
					self.barcodeType = self.barcode.type

					# draw the barcode data and barcode type on the image
					text = "{} ({})".format(self.barcodeData, self.barcodeType)
					cv2.putText(self.frame, text, (x, y - 10),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

					# if the barcode text is currently not in our CSV file, write
					# the timestamp + barcode to disk and update the set
					if self.barcodeData not in self.found:
						self.csv.write("{},{}\n".format(datetime.datetime.now(),
							self.barcodeData))
						self.csv.flush()
						self.found.add(self.barcodeData)

				# OpenCV represents images in BGR order; however PIL
				# represents images in RGB order, so we need to swap
				# the channels, then convert to PIL and ImageTk format
				image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
				image = Image.fromarray(image)
				image = ImageTk.PhotoImage(image)
		
				# if the panel is not None, we need to initialize it
				if self.panel is None:
					self.panel = tki.Label(image=image)
					self.panel.image = image
					# TODO:
					self.panel.pack(side="left", padx=30, pady=10)
		
				# otherwise, simply update the panel
				else:
					self.panel.configure(image=image)
					self.panel.image = image

		except RuntimeError, e:
			print("[INFO] caught a RuntimeError")

	def takeSnapshot(self):
		# grab the current timestamp and use it to construct the
		# output path
		ts = datetime.datetime.now()
		filename = "{}.jpg".format(ts.strftime("%Y-%m-%d_%H-%M-%S"))
		p = os.path.sep.join((self.outputPath, filename))

		# save the file
		cv2.imwrite(p, self.frame.copy())
		print("[INFO] saved {}".format(filename))

	def onClose(self):
		# set the stop event, cleanup the camera, and allow the rest of
		# the quit process to continue
		print("[INFO] closing...")
		self.stopEvent.set()
		self.vs.stop()
		self.root.quit()

	def tick(self):
		# TODO:
#		global time1
		self.time2 = time.strftime('%H:%M:%S') # local
		# TODO:
		self.time_utc = time.strftime('%H:%M:%S', time.gmtime()) # utc
		# MJD
		# TODO:
#		self.date_iso_txt = time.strftime('%Y-%m-%d') + "%.5f" % jdutil.mjd_now()
		self.date_iso_txt = time.strftime('%Y-%m-%d')
		# day, DOY, week

		# TODO:
		self.date_etc_txt = "%s DOY: %s  Week: %s" % (time.strftime('%A'), time.strftime('%j'), time.strftime('%W'))
		#self.date_etc_txt = time.strftime('%Y-%m-%d')

		if self.time2 != self.time1: # if time string has changed, update it
			self.time1 = self.time2
			self.clock_lt.config(text=self.time2)
			# TODO:
#			self.clock_utc.config(text=self.time_utc)
			self.date_iso.config(text=self.date_iso_txt)
			self.date_etc.config(text=self.date_etc_txt)

		# calls itself every 200 milliseconds
		# to update the time display as needed
		# could use &gt;200 ms, but display gets jerky
		self.clock_lt.after(20, self.tick)
