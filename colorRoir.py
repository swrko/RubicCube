import cv2
import numpy as np
import picamera
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

class Roi:
	def __init__(self, x, y, height, width, id):
		self.x = x
		self.y = y
		self.height = height
		self.width = width
		self.id = id
		self.calcCenterOfRoi()
		
	def getDim(self):
		print(self.x , " ", self.y, " ", self.height, " ", self.width)
	
	def calcCenterOfRoi(self):
		self.center = int((self.width)/2), int((self.height)/2)
	
	def getCenterOfRoi(self):
		print("center of ROI : ", self.center)
	
	def getCenterPixel(self):
		self.centerpixel = self.imgCrop[self.center]
		
	def printRoiToImage(self):
		cv2.rectangle(self.image,(self.x,self.y),(self.x + self.height, self.y + self.width),(255,0,0),2)

	def cropImageToRoiSize(self):
		self.imgCrop = self.image[self.y:self.y + self.height,self.x:self.x + self.width] # orazanie image na velkost ROI
	
	def setImage(self, image):
		self.image = image
	
	def convColorFormatOfPixel(self):
		self.hsvpixel = cv2.cvtColor(np.uint8([[self.centerpixel]]), cv2.COLOR_BGR2HSV)
		
	 # HSV -> HUE, SAT, VAL
	def maskPixel(self): # creating of masks for each color
		self.maskPixelToRed()
		self.maskPixelToLightBlue()
		self.maskPixelToGreen()
		self.maskPixelToBrown()
		self.maskPixelToWhite()
		self.maskPixelToYellow()
	 
	def maskPixelToRed(self):
		self.pixelMaskD = cv2.inRange(self.hsvpixel, np.array([0,150,90]), np.array([10,255,255])) #maska spodnej cervej farby
		self.pixelMaskU = cv2.inRange(self.hsvpixel, np.array([170,150,90]), np.array([180,255,255])) #maska vrchnej cervenej farby
		self.pixelMaskRed = self.pixelMaskD + self.pixelMaskU
		
	def maskPixelToLightBlue(self):
		self.pixelMaskLightBlue = cv2.inRange(self.hsvpixel, np.array([80,60,10]), np.array([110,255,255])) #maska lightBlue farby
		
	def maskPixelToYellow(self): 
		self.pixelMaskYellow = cv2.inRange(self.hsvpixel, np.array([25,150,90]), np.array([35,255,255])) #maska yellow farby
	
	def maskPixelToWhite(self): 
		self.pixelMaskWhite = cv2.inRange(self.hsvpixel, np.array([0,0,25]), np.array([180,50,255])) #maska white farby
	
	def maskPixelToGreen(self): 
		self.pixelMaskGreen = cv2.inRange(self.hsvpixel, np.array([50,150,10]), np.array([65,255,255])) #maska green farby
	
	def maskPixelToBrown(self): 
		self.pixelMaskBrown = cv2.inRange(self.hsvpixel, np.array([10,100,100]), np.array([20,255,200])) #maska yellow farby
			
	def setRoi(self):
		self.printRoiToImage()
		self.cropImageToRoiSize()
		self.getCenterPixel()	# get pixel
		self.convColorFormatOfPixel() # get hsv pixel format
		self.maskPixel()
	
	def getColor(self):
		# switch case for colors
		self.switch = 0
		
		# Red
		self.maskPixelToRed()
		if self.pixelMaskRed == 255:
			print(str(self.id) + ":RED")
			return
			
		# LightBlue
		self.maskPixelToLightBlue()
		if self.pixelMaskLightBlue == 255:
			print(str(self.id) + ":LightBlue")
			return
		
		# Yellow
		self.maskPixelToYellow()
		if self.pixelMaskYellow == 255:
			print(str(self.id) + ":Yellow")
			return
				
		# White
		self.maskPixelToWhite()
		if self.pixelMaskWhite == 255:
			print(str(self.id) + ":White")
			return
		
		# Green
		self.maskPixelToGreen()
		if self.pixelMaskGreen == 255:
			print(str(self.id) + ":Green")
			return
			
		# Brown
		self.maskPixelToBrown()
		if self.pixelMaskBrown == 255:
			print(str(self.id) + ":Brown")
			return

def main():
	myRoi = Roi(3,4,5,6)
	myRoi.getDim()
	myRoi.calcCenterOfRoi()
	myRoi.getCenterOfRoi()

if __name__ == "__main__":
	main() 