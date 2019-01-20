import cv2
import numpy as np
import picamera
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
#poznamka
# set camera
camera  = PiCamera()
camera.resolution = (640,480)
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.2) # warm up time 
# resize okna
cv2.namedWindow("camera",cv2.WINDOW_NORMAL)
cv2.resizeWindow("camera", 640,480)

cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
cv2.namedWindow("resolution", cv2.WINDOW_NORMAL)
cv2.resizeWindow("mask", 640,480)
cv2.resizeWindow("resolution", 640,480)

# set ROI stvorcov
[x,y,width,height]=[173,142,51,46] # hodnota prveho ROI

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format ="bgr" , use_video_port = True):
    
    image =frame.array # ulozeny frame
    
    # crop frame na roi
    imgCrop = image[y:y+height,x:x+width] # orazanie image na velkost ROI

    # vyrvorenie stvroceka pre ROI
    cv2.rectangle(image,(x,y),(x+height,y+width),(255,0,0),2)

    # maska - spodna hranica cervenej -> down
    hsv =cv2.cvtColor(imgCrop, cv2.COLOR_BGR2HSV) #konverzia
    lower_red = np.array([0,150,90])
    upper_red = np.array([10,255,255])
    maskd = cv2.inRange(hsv, lower_red, upper_red)

    # maska - vrchna hranica cervenej -> up
    hsv =cv2.cvtColor(imgCrop, cv2.COLOR_BGR2HSV) #konverzia
    lower_red = np.array([170,150,90])
    upper_red = np.array([180,255,255])
    masku = cv2.inRange(hsv, lower_red, upper_red)
    # HSV -> HUE, SAT, VAL
    
    # zjednotenie masky
    mask = maskd + masku
    
    # zjednotenie masky a originu
    res = cv2.bitwise_and(imgCrop, imgCrop, mask = mask)
    
    # zapis do okna -> zobrazenie framu
    cv2.imshow("resolution", res)
    cv2.imshow("mask", mask)
    cv2.imshow("camera", image)
    
    # zahodenie framu z camery
    rawCapture.truncate(0)
    
    # prerusenie co nejde
    key = cv2.waitKey(1) & 0xFF
    if key == None:
        break

# zavrie vsetky okna
cv2.destroyAllWindows()


