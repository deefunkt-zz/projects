import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError



img1ros = cv2.imread('TrainImage.jpg',0)          # queryImage
img2ros = cv2.imread('./frame0001.jpg',0) # trainImage

Bridge = CvBridge()

# img1cv = Bridge.imgmsg_to_cv2(img1ros,"bgr8")
img2cv = Bridge.imgmsg_to_cv2(img2ros,"bgr8")
cv2.imshow("CheckWindow",img2cv)
cv2.waitKey(3000)

# Initiate SIFT detector
orb = cv2.ORB()

# find the keypoints and descriptors with SIFT
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors.
matches = bf.match(des1,des2)

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 10 matches.
img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10], flags=2)

plt.imshow(img3),plt.show()