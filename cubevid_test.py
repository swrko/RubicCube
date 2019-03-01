import cv2
import numpy as np
import picamera
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
from PIL import Image as image
import colorRoir as cr


#poznamka
# set camera
camera  = PiCamera()
camera.resolution = (640,480)
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.2) # warm up time 
# resize okna
cv2.namedWindow("camera",cv2.WINDOW_NORMAL)
cv2.resizeWindow("camera", 640,480)

#cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
#cv2.namedWindow("resolution", cv2.WINDOW_NORMAL)
#cv2.resizeWindow("mask", 640,480)
#cv2.resizeWindow("resolution", 640,480)

# set ROI stvorcov
ROI = []
[x,y,width,height]=[160,160,80,80] # hodnota prveho ROI
X = [160, 160, 160, 240, 240, 240, 320, 320, 320]
Y = [160, 240, 320, 160, 240, 320, 160, 240, 320]
#nastavenie tresholdu +-
treshold = 20


#vytvorenie oblasti ROI
for index in range(9):
	
	ROI.append(cr.Roi(X[index], Y[index], height, width, index))



# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format ="bgr" , use_video_port = True):

	image = frame.array # ulozeny frame
	
	list(map(lambda x: x.setImage(image), ROI))
	list(map(lambda x: x.setRoi(), ROI))
	list(map(lambda x: x.getColor(), ROI))
	# init mask for each color
	
	# zapis do okna -> zobrazenie framu	
	cv2.imshow("camera", image)

	# zahodenie framu z camery
	rawCapture.truncate(0)

	# prerusenie co nejde
	key = cv2.waitKey(1) & 0xFF
	if key == None:
		break

# zavrie vsetky okna
cv2.destroyAllWindows()


