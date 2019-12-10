from __future__ import print_function
from pyimagesearch.shapedetector import ShapeDetector
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import cv2
import numpy as np
import os
import subprocess as sp
import serial
import time
import imutils
import RPi.GPIO as GPIO

def sendLineInfo(newX,oldX,width):
    try:
        if (newX != oldX): # if tempX has changed from last instance, new line has been found / line has moved
            ser.write(chr(int(newX*(126/width))+1).encode()) 
            print("writing serial value: " + str(int(newX*(126/width))+1))

    except:
        print("No new line info. No serial written.")

# Function to display camera window on GUI. Default input is True, run False to disable.
def showImage(show = True):
    if show:
        try:
            cv2.imshow("Res1", img) # Displays image windows
            cv2.imshow("mask", mask) # Displays the masked window (black and white filter)
        except:
            print("can't show camera")


# Stops thread loading images and destroys camera windows.
def cleanUp():
    cv2.destroyAllWindows()
    vs.stop()
    GPIO.cleanup()


GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)

def arduinoCallback1(channel):
    print("interrupt form arduino has been triggered!")
    #if (ser.read()==1) {
    print("Arduino detected object!")
    state = 1 # setting state = 1 to look for signs.
    #}


GPIO.add_event_detect(17, GPIO.FALLING, callback=arduinoCallback1, bouncetime=300)

# Set-up

# Defines serial at baudrate 9600
ser = serial.Serial('/dev/ttyUSB0', 9600) #ttyACM0 is default

# camera resolution (depends on camera). This can be changed to a max of 1080p, but with the downside of longer processing time.
w = 200
vs = WebcamVideoStream(src=0).start()

# Defines lower color values for color filters
lr_b = 0 #0
lg_b = 40 #40
lb_b = 190 #190

hr_b = 66 # 66
hg_b = 126 # 126
hb_b = 255 # 255

lr_r = 240 #235
lg_r = 0 #25
lb_r = 0 #50

hr_r = 255 #255
hg_r = 75 #50
hb_r = 125 #100

lr_g = 0 #235
lg_g = 160 #25
lb_g = 0 #50

hr_g = 65 #255
hg_g = 255 #50
hb_g = 145 #100

# Defines numpy array with color filter values
lower_color_blue = np.array([lb_b, lg_b, lr_b], dtype=np.uint8)
upper_color_blue = np.array([hb_b, hg_b, hr_b], dtype=np.uint8)
lower_color_green = np.array([lb_g, lg_g, lr_g], dtype=np.uint8)
upper_color_green = np.array([hb_g, hg_g, hr_g], dtype=np.uint8)
lower_color_red = np.array([lb_r, lg_r, lr_r], dtype=np.uint8)
upper_color_red = np.array([hb_r, hg_r, hr_r], dtype=np.uint8)

print("blyat")
max_slider = 40

# current robot line coordinates is defined with null values.
hx1 = 0 
hx2 = 0
hy1 = 0
hy2 = 0

packageSymbol = "rectangle"
# highLineY is a temporary value which remembers max Y value of previous line. A line can not be selected unless it has a higher Y value than this line.
# This variable is slowly decreased in the code if the robot does not detect any valid lines, until it eventually reaches 0 and the robot will detect and blue line it sees.
highLineY = 0
tempX = 320
old_tempX = 320
state = 1 #0: following line, 1: looking for sign, 2: picking up package, 3: delivering package
# While loop for main logic
while True:        
    # Image parameters / set-up for selecting colors and finding lines
    img = vs.read()
    img = imutils.resize(img, width=w)
    
    if state == 0:
        mask = cv2.inRange(img, lower_color_blue, upper_color_blue) # find colors between the color limits defined earlier. This image is black and white.
        edges = cv2.Canny(mask,50,100) # Find edges from the previously defined mask.
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, max_slider, minLineLength=50, maxLineGap=100) # This command finds lines from the edges found previously. Lines becomes an array of line start/end coordinates

        try:
            for index, line in enumerate(lines): # This for-loop finds the line with the highest (lowest on screen) Y-coordinate. This will become the line the robot will follow as it's the line closest to the robot.
                #print("a")
                x1, y1, x2, y2 = line[0]
                if (y1 > y2):
                    if (y1 > highLineY): # If new Y coordinate is higher than previously highest Y coordinate, dump previous line and replace its line coordinates with the newly found line.
                        #print("")
                        hx1 = x1
                        tempX = x1
                        hx2 = x2
                        hy1 = y1
                        hy2 = y2
                        highLineY = y1
                elif (y2 > y1):
                    if (y2 > highLineY):
                        #print("")
                        hx1 = x1
                        hx2 = x2
                        tempX = x2
                        hy1 = y1
                        hy2 = y2
                        highLineY = y2
                else:
                    #print("what")
                    cv2.line(img, (hx1, hy1), (hx2, hy2), (255, 0, 255), 5)
            if (highLineY > 10): # This slowly reduces the previously highest Y coordinate. This mechanism is neccesary as the robot would otherwise quickly select a new totally different line, if it for a moment can't see it's previous line.
                highLineY = highLineY - 5
            cv2.line(img, (hx1, hy1), (hx2, hy2), (255, 0, 255), 5) # Draws the new line on the "img" windows.
            # chr(254).encode()
            #ser.write(chr(tempX*0.39).encode())
            #print(chr(tempX*0.39).encode())
        
        except:
            #print("can't find line.")
            if (highLineY > 10):
                highLineY = highLineY - 5
        sendLineInfo(newX = tempX, oldX = old_tempX, width = w)
        
        old_tempX = tempX
    elif state == 1:
        print("spam1")
        green = 0
        red = 0
        compare = 0
        for i in range(10):
            for l in range(2):
                image = vs.read()
                if l == 0:
                    mask = cv2.inRange(image, lower_color_green, upper_color_green)
                elif l == 1:
                    mask = cv2.inRange(image, lower_color_red, upper_color_red)
                resized = imutils.resize(mask, width=300)
                ratio = mask.shape[0] / float(resized.shape[0])

                # convert the resized image to grayscale, blur it slightly,
                # and threshold it
                #gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
                gray = resized
                blurred = cv2.GaussianBlur(gray, (7, 7), 0)
                thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1] #60, 255 default

                # find contours in the thresholded image and initialize the
                # shape detector
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                sd = ShapeDetector()

                try:
                    # loop over the contours
                    for c in cnts:
                        # compute the center of the contour, then detect the name of the
                        # shape using only the contour
                        M = cv2.moments(c)
                        cX = int((M["m10"] / M["m00"]) * ratio)
                        cY = int((M["m01"] / M["m00"]) * ratio)
                        shape = sd.detect(c)

                        if (shape==packageSymbol and l==0):
                            green = green + 1
                        elif (shape==packageSymbol and l==1):
                            red = red + 1

                        compare = compare + 1

                        # multiply the contour (x, y)-coordinates by the resize ratio,
                        # then draw the contours and the name of the shape on the image
                        c = c.astype("float")
                        c *= ratio
                        c = c.astype("int")
                        cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
                        cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 0, 0), 2)
                except:
                    pass

                cv2.imshow("Image1", image)
                cv2.imshow("Mask", mask)
                cv2.imshow("test1",thresh)

                time.sleep(.100)
        #analyse colors here..
        print("spam2")
        if compare > 30:
            if compare / (green+1) > 3:
                print("turn right")
            elif compare / (red+1) > 3:
                print("turn left")
        state = 0

    showImage() # run showImage(False) to disable imageview.

    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(chr(int(0)).encode()) # sending 0 over serial to stop movement.
        break # Stops program if button "q" is pressed.
        
cleanUp()