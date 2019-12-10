from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import cv2
import numpy as np
import os
import subprocess as sp
import serial
import time
import argparse
import imutils

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



# Set-up

# Defines serial at baudrate 9600
ser = serial.Serial('/dev/ttyUSB0', 9600) #ttyACM0 is default

# camera resolution (depends on camera). This can be changed to a max of 1080p, but with the downside of longer processing time.
w = 200
#cap = cv2.VideoCapture(0) # Chooses camera to use for image recognition (2 on pc, probably 0 or 1 on RPi)
#cap.set(3,width)
#cap.set(4,height)
vs = WebcamVideoStream(src=0).start()

# Defines lower color values for color filter. (blue currently)
lr = 0 #0
lg = 200 #40
lb = 110 #190

# Defines upper color values for color filter. (blue currently)
hr = 40 # 66
hg = 255 # 126
hb = 160 # 255

# Defines numpy array with color filter values
lower_color = np.array([lb, lg, lr], dtype=np.uint8)
upper_color = np.array([hb, hg, hr], dtype=np.uint8)
max_slider = 40
# current robot line coordinates is defined with null values.
hx1 = 0 
hx2 = 0
hy1 = 0
hy2 = 0

# highLineY is a temporary value which remembers max Y value of previous line. A line can not be selected unless it has a higher Y value than this line.
# This variable is slowly decreased in the code if the robot does not detect any valid lines, until it eventually reaches 0 and the robot will detect and blue line it sees.
highLineY = 0
tempX = 320
old_tempX = 320

# While loop for main logic
while True:
    # Image parameters / set-up for selecting colors and finding lines
    #ret, img = cap.read()
    img = vs.read()
    img = imutils.resize(img, width=w)
    
    mask = cv2.inRange(img, lower_color, upper_color) # find colors between the color limits defined earlier. This image is black and white.
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
        print("IT WORKS...")
        if (highLineY > 10):
            highLineY = highLineY - 5
            
    sendLineInfo(newX = tempX, oldX = old_tempX, width = w)


        
    showImage() # run showImage(False) to disable imageview.
        
        
    old_tempX = tempX

    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(chr(int(0)).encode()) # sending 0 over serial to stop movement.
        break # Stops program if button "q" is pressed.
        
cleanUp()




# Definitions of functions

# Function to send line info if line position has changed

