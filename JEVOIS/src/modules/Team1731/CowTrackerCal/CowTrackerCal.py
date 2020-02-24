# ###################################################################################################
## FRC Team 1731 - Fresta Valley Robotics Club
## Originally written by FRC Team 2073 - EagleForce
## INFINITE RECHARGE - 2020

## Summary:
## Camera-side code that calibrates the CowTracker so that the binary image, which determines where 
## targets are drawn, only shows actual targets you want.
## These values are written to a calibration.txt file in data/cowtracker/

## ENSURE YOU CHANGE THE serdev PARAMETER IN COWTUNER TO CALIBRATE!!!
# ###################################################################################################

# These libraries are built-in to the JeVois camera, so the Python compiler will throw errors. Ignore these.
import libjevois as jevois
import cv2
import numpy as np

# The starting values to write to the calibration file
upperHue = 95
lowerHue = 60
upperSat = 255
lowerSat = 100
upperVal = 255
lowerVal = 100
errode = 0
dilate = 0
approx = 9
area = 100
solidity = 100

class CowTrackerCal:
	# ###################################################################################################
	## Constructor
	def __init__(self):
		# Instantiate a JeVois Timer to measure our processing framerate:
		self.timer = jevois.Timer("Catbox", 100, jevois.LOG_INFO)
		
	# ###################################################################################################
	## Process function with USB output
	def process(self, inframe, outframe):
		# Global variables
		global upperHue
		global lowerHue
		global upperSat
		global lowerSat
		global upperVal
		global lowerVal
		global errode
		global dilate
		global approx
		global area
		global solidity

		# Get the next camera image (may block until it is captured) and here convert it to OpenCV BGR by default. If
		# you need a grayscale image instead, just use getCvGRAY() instead of getCvBGR(). Also supported are getCvRGB()
		# and getCvRGBA():
		inimg = inframe.getCvBGR()
		
		# Start measuring image processing time (NOTE: does not account for input conversion time): 
		#Truely useless and can be removed
		self.timer.start()
		#Convert the image from BGR(RGB) to HSV
		hsvImage = cv2.cvtColor( inimg, cv2.COLOR_BGR2HSV)
		
		## Threshold HSV Image to find specific color
		binImage = cv2.inRange(hsvImage, (lowerHue, lowerSat, lowerVal), (upperHue, upperSat, upperVal))
		
		# Erode image to remove noise if necessary.
		binImage = cv2.erode(binImage, None, iterations = errode)
		#Dilate image to fill in gaps
		binImage = cv2.dilate(binImage, None, iterations = dilate)
		
		#This image is used to display the thresholded image. Bounding Rectangle is added below.
		#Use this image to tune your targeting parameters.
		binOut = cv2.cvtColor(binImage, cv2.COLOR_GRAY2BGR)
		
		##Finds contours (like finding edges/sides), 'contours' is what we are after
		# im2, contours, hierarchy = cv2.findContours(binImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
		contours, hierarchy = cv2.findContours(binImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_KCOS)
		
		##arrays to will hold the good/bad polygons
		squares = []
		badPolys = []
		
		## Parse through contours to find targets
		for c in contours:
			if (contours != None) and (len(contours) > 0):
				cnt_area = cv2.contourArea(c)
				hull = cv2.convexHull(c , 1)
				hull_area = cv2.contourArea(hull)  #Used in Solidity calculation
				p = cv2.approxPolyDP(hull, approx, 1)
				if (cv2.isContourConvex(p) != False) and (len(p) == 4) and (cv2.contourArea(p) >= area): #p=3 triangle,4 rect,>=5 circle
					filled = cnt_area/hull_area
					if filled <= solidity: #Used to determine if target is hollow or not
						squares.append(p)
				else:
					badPolys.append(p)
		
		##BoundingRectangles are just CvRectangles, so they store data as (x, y, width, height)
		##Calculate and draw the center of the target based on the BoundingRect
		for s in squares:		 
			br = cv2.boundingRect(s)
			#Target "x" and "y" center 
			x = br[0] + (br[2]/2)
			y = br[1] + (br[3]/2)
			cv2.rectangle(binOut, (br[0],br[1]),((br[0]+br[2]),(br[1]+br[3])),(0,0,255), 2,cv2.LINE_AA)

		# Convert our BGR output image to video output format and send to host over USB. If your output image is not
		# BGR, you can use sendCvGRAY(), sendCvRGB(), or sendCvRGBA() as appropriate:
		outframe.sendCvBGR(binOut)

		# Write calibration values to a txt file. Store it in share so that other modules can use them
		# Hopefully storing it in share will stop the access permissions errors
		CalFile = open('data/cowtracker/calibration.txt', 'w')
		CalFile.truncate()#Clear out old calibraton values
		CalFile.write("MaxHue: "+str(upperHue))
		CalFile.write(" ,")
		CalFile.write("MinHue: "+str(lowerHue))
		CalFile.write(" ,")
		CalFile.write("MaxSaturation: "+str(upperSat))
		CalFile.write(" ,")
		CalFile.write("MinSaturation: "+str(lowerSat))
		CalFile.write(" ,")
		CalFile.write("MaxValue: "+str(upperVal))
		CalFile.write(" ,")
		CalFile.write("MinValue: "+str(lowerVal))
		CalFile.write(" ,")
		CalFile.write("Errode: "+str(errode))
		CalFile.write(" ,")
		CalFile.write("Dilate: "+str(dilate))
		CalFile.write(" ,")
		CalFile.write("Approximate: "+str(approx))
		CalFile.write(" ,")
		CalFile.write("Area: "+str(area))
		CalFile.write(" ,")
		CalFile.write("Solidity (percent): "+str(solidity))
		
		CalFile.close()#Close calibration file
		
	# ###################################################################################################
	## Parse a serial command forwarded to us by the JeVois Engine, return a string
	def parseSerial(self, cmd):
		# Global variables
		global upperHue
		global lowerHue
		global upperSat
		global lowerSat
		global upperVal
		global lowerVal
		global errode
		global dilate
		global approx
		global area
		global solidity
		
		jevois.LINFO("parseserial received command [{}]".format(cmd))
		
		cal = cmd.split("=")
		
		if cal[0] == "lowerHue":
			lowerHue = int(cal[1])
			return cal[1]
		
		if cal[0] == "upperHue":
			upperHue = int(cal[1])
			return cal[1]
			
		if cal[0] == "upperSat":
			upperSat = int(cal[1])
			return cal[1]
			
		if cal[0] == "lowerSat":
			lowerSat = int(cal[1])
			return cal[1]
			
		if cal[0] == "upperVal":
			upperVal = int(cal[1])
			return cal[1]
			
		if cal[0] == "lowerVal":
			lowerVal = int(cal[1])
			return cal[1]
			
		if cal[0] == "errode":
			errode = int(cal[1])
			return cal[1]
			
		if cal[0] == "dilate":
			dilate = int(cal[1])
			return cal[1]
			
		if cal[0] == "approx":
			approx = int(cal[1])
			return cal[1]
			
		if cal[0] == "area":
			area = int(cal[1])
			return cal[1]
			
		if cal[0] == "solidity":
			solidity = int(cal[1])/100
			return cal[1]
			
		return "ERR: Fat Fingered that command"
		
			
			
	# ###################################################################################################
	## Return a string that describes the custom commands we support, for the JeVois help message
	def supportedCommands(self):
		# use \n seperator if your module supports several commands
		return "Use the CowTuner.py script on a RPi, PC, etc. to send Serial commands to tune the vision tracking."
		


