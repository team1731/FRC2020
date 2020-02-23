# ###################################################################################################
## FRC Team 1731 - Fresta Valley Robotics Club
## Originally written by FRC Team 2073 - EagleForce
## INFINITE RECHARGE - 2020

## Summary:
## CowTracker finds the half-hexagon target on the top power port and draws a bounding box around it.
## It then gets the center point of the bounding box and normalizes such that 0 is the center of the
## camera. These values are then sent over the serial port on the JeVois camera as a JSON string.

## This is the NO STREAM version of CowTracker. The only output from the camera is the JSON string.
# ###################################################################################################

# These libraries are built-in to the JeVois camera, so the Python compiler will throw errors. Ignore these.
import libjevois as jevois
import cv2
import numpy as np
# The rest of the imports are Python libraries

import json
import math
import time

# Camera Field Of View in degrees. JeVois = 65.0, Lifecam HD-300 = 68.5, Cinema = 73.5
FOV = 65.0

# Determines if target data is sent or not. Controlled by serial commands SEND and STOP (see parseSerial(cmd))
sendTargetData = True

# Threshold values for Trackbars. These were loaded from a calibration file, but our JeVois OS kept changing permissions.
# CowTrackerCal will write a calibration file. These values should be stored as constants as we have done below.
# CowTrackerCal also has changes in how it writes these values, so the below code will not work
'''
rawCalFile = open ('data/cowtracker/calibration.txt', 'r')
readCalFile = rawCalFile.read()
CalFile = readCalFile.split(",")
uh = int(CalFile[0])
lh = int(CalFile[1])
us = int(CalFile[2])
ls = int(CalFile[3])
uv = int(CalFile[4])
lv = int(CalFile[5])
er = int(CalFile[6])
dl = int(CalFile[7])
ap = int(CalFile[8])
ar = int(CalFile[9])
sl = float(CalFile[10])

rawCalFile.close() #Close calibration file
'''

# Constant calibration files that are manually copied from the calibration file
uh = 95
lh = 60
us = 255
ls = 100
uv = 255
lv = 100
er = 0
dl = 0
ap = 9
ar = 100
sl = 1.0

# ###################################################################################################
## Stores data from targets
class TargetObject:
	def __init__(self, _xPos, _yPos, _area):
		self.xPos = _xPos
		self.yPos = _yPos
		self.area = _area

class CowTracker:
	# ###################################################################################################
	## Constructor
	def __init__(self):
		# Instantiate a JeVois Timer to measure our processing framerate:
		self.timer = jevois.Timer("Catbox", 100, jevois.LOG_INFO)
		
	# ###################################################################################################
	## Process function with USB output. This the NO STREAM version, so this function will attempt to pass it through processNoUSB
	def process(self, inframe, outframe):
		jevois.LINFO("CowTracker should not be used with USB output!")
		self.processNoUSB(inframe)

	# ###################################################################################################
	## Process function without USB output
	def processNoUSB(self, inframe):

		# Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR by default. If
		# you need a grayscale image instead, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB()
		# and getCvRGBA():
		
		inimg = inframe.getCvBGR()
		
		# Start measuring image processing time (NOTE: does not account for input conversion time):
		self.timer.start()
		myTimer = time.time()

		# Convert the image from BGR(RGB) to HSV
		hsvImage = cv2.cvtColor( inimg, cv2.COLOR_BGR2HSV)
		
		# Threshold HSV Image to find specific color
		binImage = cv2.inRange(hsvImage, (lh, ls, lv), (uh, us, uv))
		
		# Erode image to remove noise if necessary.
		binImage = cv2.erode(binImage, None, iterations = er)

		# Dilate image to fill in gaps
		binImage = cv2.dilate(binImage, None, iterations = dl)
		
		# Finds contours (like finding edges/sides). findContours returns hierarchy as well, but that won't be used.
		contours, hierarchy = cv2.findContours(binImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
		
		# TargetObjects holds an array of TargetObjects.
		targetObjects = []

		# Parse through contours to find valid targets
		for index in range(len(contours)):
			c = contours[index]
			if (contours != None) and (len(contours) > 0):
				# Solidity calculation
				cnt_area = cv2.contourArea(c)
				hull = cv2.convexHull(c , 1)
				hull_area = cv2.contourArea(hull)
				p = cv2.approxPolyDP(hull, ap, 1)
				
				if (cv2.isContourConvex(p) != False) and (len(p) == 4) and (cv2.contourArea(p) >= ar):
					filled = cnt_area/hull_area
					if filled <= sl:
						
						# Find center point of the bounding box
						br = cv2.boundingRect(p)
						x = br[0] + (br[2]/2)
						y = br[1] + (br[3]/2)
						
						# Add the target to the array
						targetObjects.append(TargetObject(x, y, cv2.contourArea(c)))

						# Calculate points for drawing contours. 
						# Used only in the CowTrackerDebug.
						'''
						rect = cv2.minAreaRect(p)
						box = cv2.boxPoints(rect)
						box = np.int0(box)
						cv2.drawContours(inimg,[box],0,(255,0,0),1)
						cv2.putText(inimg, str(index), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255),1, cv2.LINE_AA)
						'''	

		outimg = inimg
		imgHeight, imgWidth, imgChannels = outimg.shape

		# Set a tallest target so that it is impossible for another target to be shorter
		shortestTarget = TargetObject(imgWidth, imgHeight, 0)

		# Only perform work if there are any targets visible
		if len(targetObjects) > 0:

			# Find the tallest target as we are not going to aim for the low power port
			for targetObject in targetObjects:

				# y = 0 is at the top of the camera. So the higher the y position, the lower the object is
				if targetObject.yPos < shortestTarget.yPos:
					shortestTarget = targetObject
			
			# Draw debug the highest target.
			# Used only in CowTrackerDebug
			#if shortestTarget.area != 0:
				#cv2.circle(inimg, (int(shortestTarget.xPos), int(shortestTarget.yPos)), 10, (0,255,255), 1)

		
		
		
		# Normalize coordinates such that 0.5 is center, and 0 & 1 are edges
		normalizedY = 0
		normalizedZ = 0

		# Make sure that somehow, the highest target wasn't at the bottom
		if shortestTarget.area != 0:
			focalLength = imgWidth/(2 * math.tan(math.radians(FOV)/2))
			normalizedY = -(shortestTarget.xPos - (imgWidth/2 - 0.5)) / focalLength
			normalizedZ = (shortestTarget.yPos - (imgHeight/2 - 0.5)) / focalLength

		# Calculate time spent calculating cause that's funny (NOTE: does not account for output conversion time)
		# Used only in CowTrackerDebug
		#fps = self.timer.stop()
		#cv2.putText(outimg, fps, (3, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

		# Build JSON string and send it out the serial port if we are sending data
		global sendTargetData
		if sendTargetData:
			pixels = {"DeltaTime" : str((time.time()-myTimer)), "Y" : str(normalizedY), "Z" : str(normalizedZ)}
			jevois.sendSerial(json.dumps(pixels))
		
		# Convert our BGR output image to video output format and send to host over USB. If your output image is not
		# BGR, you can use sendCvGRAY(), sendCvRGB(), or sendCvRGBA() as appropriate:
		# Used only in CowTrackerDebug
		#outframe.sendCvBGR(outimg,50)

	# ###################################################################################################
	## Parse a serial command forwarded to us by the JeVois Engine, return a string response
	def parseSerial(self, cmd):
		# cmd is the raw string that was sent over the serial
		cmd = cmd.upper()

		# Change the sendTargetData depending on the serial command received
		global sendTargetData
		if cmd == "SEND":
			sendTargetData = True
			return "SENDING DATA STREAM TO ROBORIO"
		elif cmd == "STOP":
			sendTargetData = False
			return "STOPPING DATA STREAM"

		# If the serial command is not handled by this module, let the JeVois handle the command
		return "NOT A MODULE COMMAND"

