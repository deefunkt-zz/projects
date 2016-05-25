# import the necessary packages
from collections import deque   # a data structure?
import numpy as np
import argparse
import imutils
import Image
import cv2



# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
blueLower = (53, 21, 16)  # BGR
blueUpper = (100, 51, 43)
# pts = deque(maxlen=args["buffer"])

Im = cv2.imread('frame0000.jpg')
# Imres = imutils.resize(Im, width=600)
cv2.imshow("Image window", Im)
cv2.waitKey(5)
# keep looping
while True:
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	# hsv = cv2.cvtColor(Im, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(Im, blueLower, blueUpper)
	cv2.imshow("Image window", mask)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(Im, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(Im, center, 5, (0, 0, 255), -1)

	# update the points queue
	pts.appendleft(center)

