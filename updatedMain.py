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

#sets up variables:
ser = serial.Serial('/dev/ttyACM0', 9600)
stateDelivering = False
lowPower = 0
lookingForSign = 0
stateChecking = 0
print("starting python")
print("sleeping for 2 sec...")

#sets up threaded image loading and custom picture width "w"
w = 200
vs = WebcamVideoStream(src=0).start()
 
# Defines color filters for different color ranes
lr_b = 0
lg_b = 120
lb_b = 190

hr_b = 50
hg_b = 230
hb_b = 255


############## OLD VALUES USED
#lr_b = 0
#lg_b = 130
#lb_b = 160

#hr_b = 130
#hg_b = 255
#hb_b = 255


lr_r = 200
lg_r = 0
lb_r = 0

hr_r = 255 
hg_r = 100 
hb_r = 145

lr_g = 0
lg_g = 125
lb_g = 0

hr_g = 130
hg_g = 255
hb_g = 165

# Defines numpy array with color filter values
lower_color_blue = np.array([lb_b, lg_b, lr_b], dtype=np.uint8)
upper_color_blue = np.array([hb_b, hg_b, hr_b], dtype=np.uint8)
lower_color_green = np.array([lb_g, lg_g, lr_g], dtype=np.uint8)
upper_color_green = np.array([hb_g, hg_g, hr_g], dtype=np.uint8)
lower_color_red = np.array([lb_r, lg_r, lr_r], dtype=np.uint8)
upper_color_red = np.array([hb_r, hg_r, hr_r], dtype=np.uint8)

#image filtering parameter
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

# Sets up GPIO (hardware pins on raspberrypi)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)

GPIO.setup(18, GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
GPIO.outout(27,0)
servoPWM = GPIO.PWM(18, 50) # channel 0, 50hz PWM frequency.
servoPWM.start(93.2)
servoPWM.ChangeDutyCycle(93.2)

time.sleep(2)

def sendLineInfo(newX,oldX,width):
    try:
        if (newX != oldX): # if tempX has changed from last instance, new line has been found / line has moved
            ser.write(chr(int(newX*(126/width))+1).encode()) 
            #print("writing serial value: " + str(int(newX*(126/width))+1)).

    except:
        print("No new line info. No serial written.")

def cleanUp():
    cv2.destroyAllWindows()
    vs.stop()
    GPIO.cleanup()

def arduinoCallback1(channel):
    global lookingForSign
    global lowPower
    global stateDelivering
    global count
    global stateChecking
    ser.reset_output_buffer()
    print("interrupt triggered...")
    read = ser.read()
    time.sleep(1)
    if read == b'\x01':
        lookingForSign = 1
        stateChecking = 0
        print("metal detected.")
        #ser.write(chr(int(3)).encode())####################################################################################################################################
    elif read == b'\x02':     
        stateChecking = 1
        lookingForSign = 0

GPIO.add_event_detect(17, GPIO.FALLING, callback=arduinoCallback1, bouncetime=2000)

 #0: following line, 1: looking for sign, 2: picking up package, 3: delivering package.
# While loop for main logic


while True:
    packageSymbol = "triangle" # to predefine packagename, enter something other than ""
    image = vs.read()
    image = imutils.resize(image, width=w)
    # Image parameters / set-up for selecting colors and finding liness
    if stateChecking == 1:
        if packageSymbol == "":
            #TURN CAMERA UP
            compare = 0
            foundSign = 0
            triangles = 0
            squares = 0

            for i in range(10):
                image = vs.read()
                
                mask1 = cv2.inRange(image, lower_color_red, upper_color_red)
                resized = imutils.resize(mask1, width=300)
                ratio = mask1.shape[0] / float(resized.shape[0])

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

                        if shape=="triangle":
                            triangles += 1
                        elif shape =="rectangle" or shape == "square":
                            squares += 1
                        compare += 1

                        # multiply the contour (x, y)-coordinates by the resize ratio,
                        # then draw the contours and the name of the shape on the image
                        c = c.astype("float")
                        c *= ratio
                        c = c.astype("int")
                        #cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
                        #cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                        #    0.5, (0, 0, 0), 2)

                        
                except:
                    pass
                cv2.imshow("thresh",thresh)
                #cv2.imshow("Image1", image)
                #cv2.imshow("mask1", mask1)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    ser.write(chr(int(0)).encode()) # sending 0 over serial to stop movement.
                    break # Stops program if button "q" is pressed.
                time.sleep(.200)
            #analyse colors here..
            print("detected following shapes:")
            print(compare)
            print(squares)
            print(triangles)

            if compare < 20 and compare > 6:
                if triangles > squares:
                    packageSymbol = "triangle"
                    print("detected triangle!")
                if triangles < squares:
                    packageSymbol = "rectangle"
                    print("detected rectangle!")
                #stateChecking = 0
                # MOVE CAMERA DOWN
            else:
                print("didn't find sign, checking again...")

            if not packageSymbol == "":
                stateChecking = 0
                GPIO.outout(27,1)
                ser.write(chr(int(63)))
                time.sleep(0.5)
                GPIO.outout(27,0)
                ser.write(chr(int(0)))
        if not packageSymbol == "":
            print("package already defined as: " + packageSymbol)
            stateChecking = 0
            GPIO.outout(27,1)
            ser.write(chr(int(63)))
            time.sleep(0.5)
            GPIO.outout(27,0)
            ser.write(chr(int(0)))


    elif lookingForSign == 1:
        green = 0
        red = 0
        compare = 0
        for i in range(10):
            for l in range(2):
                image = vs.read()

                if l == 0:
                    mask1 = cv2.inRange(image, lower_color_green, upper_color_green)
                elif l == 1:
                    mask1 = cv2.inRange(image, lower_color_red, upper_color_red)
                resized = imutils.resize(mask1, width=300)
                ratio = mask1.shape[0] / float(resized.shape[0])

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
                        #cv2.drawContours(image, [c], -1, (0, 0, 255), 2)
                        #cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                        #    0.5, (0, 0, 0), 2)
                        
                except:
                    pass

                cv2.imshow("thresh",thresh)
                #cv2.imshow("Image1", image).
                #cv2.imshow("mask1", mask1)
                time.sleep(.200)
        #analyse colors here..
        print("shape info:")
        print(compare)
        print(green)
        print(red)

        if compare > 30 and compare < 55:
            if (compare / (green+1)) < 5:
                print("turning right...")
                GPIO.outout(27,1)
                ser.write(chr(int(126)).encode()) 
                lookingForSign = 0
                time.sleep(0.6)
                GPIO.outout(27,0)
                packageSymbol = ""
            elif (compare / (red+1)) < 5:
                print("turn left...")
                GPIO.outout(27,1)
                ser.write(chr(int(2)).encode())
                lookingForSign = 0
                time.sleep(0.6)
                GPIO.outout(27,0)
                packageSymbol = ""
            else:
                print("bad shapes...")
        else:
            print("too many or not enough!")

    elif lookingForSign == 0 and stateChecking == 0:
        
        mask = cv2.inRange(image, lower_color_blue, upper_color_blue) # find colors between the color limits defined earlier. This image is black and white.

        edges = cv2.Canny(mask,50,100) # Find edges from the previously defined mask.
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, max_slider, minLineLength=60, maxLineGap=100) # This command finds lines from the edges found previously. Lines becomes an array of line start/end coordinates

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
                    cv2.line(image, (hx1, hy1), (hx2, hy2), (255, 0, 255), 5)
            if (highLineY > 10): # This slowly reduces the previously highest Y coordinate. This mechanism is neccesary as the robot would otherwise quickly select a new totally different line, if it for a moment can't see it's previous line.
                highLineY = highLineY - 5
            
            cv2.line(image, (hx1, hy1), (hx2, hy2), (255, 0, 255), 5) # Draws the new line on the "img" windows.
        
        except:
            #print("can't find line.")
            if (highLineY > 10):
                highLineY = highLineY - 5

        if not stateChecking and not lookingForSign:        
            sendLineInfo(newX = tempX, oldX = old_tempX, width = w)
        
        old_tempX = tempX

        try:
            cv2.imshow("Res1", image) # Displays image windows
            cv2.imshow("mask", mask) # Displays the masked window (black and white filter)
        except:
            print("can't show camera...")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(chr(int(0)).encode()) # sending 0 over serial to stop movement.
        break # Stops program if button "q" is pressed.
        
cleanUp()