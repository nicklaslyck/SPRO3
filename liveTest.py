# USAGE
# python detect_shapes.py --image shapes_and_colors.png

# import the necessary packages
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

# construct the argument parse and parse the arguments

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better


# following upper and lower values are for red shapes on printed paper without assisting lights.
if (False):

    lr = 240 #235
    lg = 0 #25
    lb = 0 #50

    # Defines upper color values for color filter. (blue currently)
    hr = 255 #255
    hg = 75 #50
    hb = 125 #100
else: 
    
    lr = 0 #235
    lg = 160 #25
    lb = 0 #50

    # Defines upper color values for color filter. (blue currently)
    hr = 50 #255
    hg = 255 #50
    hb = 145 #100

lower_color = np.array([lb, lg, lr], dtype=np.uint8)
upper_color = np.array([hb, hg, hr], dtype=np.uint8)

selected = "triangle"
w = 640

#image = cv2.imread(args["image"])
vs = WebcamVideoStream(src=0).start()

while True:

    confidence = 0
    compare = 0
    for i in range(10):
        image = vs.read()
        mask = cv2.inRange(image, lower_color, upper_color)
        
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
                if (shape=="triangle"):
                    confidence = confidence + 1
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
        cv2.imshow("test",thresh)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(.100)
    print(confidence)
    print(compare)
    
cv2.destroyAllWindows()
vs.stop()