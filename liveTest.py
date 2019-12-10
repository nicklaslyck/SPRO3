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


lr_r = 240 #235
lg_r = 0 #25
lb_r = 0 #50

hr_r = 255 #255
hg_r = 75 #50
hb_r = 125 #100

lr_g = 0 #235
lg_g = 160 #25
lb_g = 0 #50

hr_g = 50 #255
hg_g = 255 #50
hb_g = 145 #100

lower_color_green = np.array([lb_g, lg_g, lr_g], dtype=np.uint8)
upper_color_green = np.array([hb_g, hg_g, hr_g], dtype=np.uint8)
lower_color_red = np.array([lb_r, lg_r, lr_r], dtype=np.uint8)
upper_color_red = np.array([hb_r, hg_r, hr_r], dtype=np.uint8)

selected = "triangle"
w = 640

#image = cv2.imread(args["image"])
vs = WebcamVideoStream(src=0).start()

while True:

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

                    if (shape==selected and l==0):
                        green = green + 1
                    elif (shape==selected and l==1):
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

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(.100)
    print("_________")
    print(red)
    print(green)
    print(compare)
    
cv2.destroyAllWindows()
vs.stop()