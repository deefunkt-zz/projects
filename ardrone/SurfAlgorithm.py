import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import Image

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#
# T = Image.open('totrain.png')
# I = Image.open('MewithMarker.jpg')
# train = cv2.imread('/home/astrochick/Documents/projects/Train1_Lyd.jpg',1)          # queryImage
# imgcopy = cv2.imread('/home/astrochick/Documents/projects/Lyd1.jpg',1) # trainImage
# imgcopy2 = cv2.imread('/home/astrochick/Documents/projects/Lyd1.jpg',1)
train = cv2.imread('/home/astrochick/Documents/projects/Train_mandalasfar.jpg',1)          # queryImage
imgcopy = cv2.imread('/home/astrochick/Documents/projects/Train_mandalas.jpg',1) # trainImage
# imgcopy2 = cv2.imread('/home/astrochick/Documents/projects/Lyd1.jpg',1)

# img1cv = Bridge.imgmsg_to_cv2(img1ros,"bgr8")
# cv2.imshow("CheckWindow",train)
# cv2.waitKey(3000)

# Initiate SIFT detector
# orb = cv2.ORB(nfeatures=50,scaleFactor=1.2,nlevels=15,edgeThreshold=0,WTA_K=2,patchSize=31)
orb = cv2.ORB()

# find the keypoints and descriptors with SIFT
kp1, des1 = orb.detectAndCompute(train,None)
kp2, des2 = orb.detectAndCompute(imgcopy,None)
            # Match descriptors.
# matches = bf.match(des1,des2)
# matches = sorted(matches, key=lambda x:x.distance)
# match10 = matches[:10]

FLANN_INDEX_LSH = 6
index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 12, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params,search_params)
good = flann.knnMatch(des1,des2,k=4)
# matchesMask = [[0,0] for i in xrange(len(matches))]
good = []
for m_n in matches:
    if len(m_n) != 2:
        continue
    (m,n) = m_n
    if m.distance < 0.75*n.distance:
        good.append(m)

# Apply ratio test
# good = []
# for m,n in matches:
#     if m.distance < 0.75*n.distance:
#         good.append(m)
# sets = matches[:50]
src_pts = np.float32([ kp1[m[0].queryIdx].pt for m in good]).reshape(-1,1,2)
src_pts2 = np.float32([ kp1[m[1].queryIdx].pt for m in good]).reshape(-1,1,2)
dst_pts = np.float32([ kp2[m[0].trainIdx].pt for m in good]).reshape(-1,1,2)
dst_pts2 = np.float32([ kp2[m[1].trainIdx].pt for m in good]).reshape(-1,1,2)

size = imgcopy.shape
bareim = np.zeros(size[:2],np.uint8)
index = totuple(np.int32(dst_pts).reshape(-1,2))
for i in index:
    cv2.circle(bareim,i,5,(255),5)
index = totuple(np.int32(dst_pts2).reshape(-1,2))
for i in index:
    cv2.circle(bareim,i,5,(150),5)

cv2.imshow("mask",bareim)
cv2.waitKey(3000)
bareim = cv2.erode(bareim, None, iterations=2)
bareim = cv2.dilate(bareim, None, iterations=2)
cnts = cv2.findContours(bareim.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
center = np.array([0,0])
if len(cnts) > 0:  # only proceed if at least one contour was found
    c = max(cnts, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    M = cv2.moments(c)
    center[0] = int(M["m10"] / M["m00"])
    center[1] = int(M["m01"] / M["m00"])
    cv2.circle(imgcopy,tuple(center),3,(255,0,0),3)
cv2.imshow("window",imgcopy)
cv2.waitKey(2000)


# while 1:
#     pass




# matchesMask = mask.ravel().tolist()

src_int = totuple(np.int32([src_pts]).reshape(-1,2))
dst_int = totuple(np.int32([dst_pts]).reshape(-1,2))
for i in dst_int:
    cv2.circle(imgcopy,i,2,(255,0,0),-1)
    # cv2.imshow("compare",imgcopy)
    # cv2.waitKey(1000)
for i in src_int:
    cv2.circle(train,i,2,(255,0,0),-1)
    # cv2.imshow("compare1",train)
    # cv2.waitKey(500)

# cv2.circle(imgcopy,points[6],3,(0,0,255),2)
# cv2.imshow("compare",imgcopy)
# cv2.waitKey(3000)
# cv2.circle(train,src_int[6],3,(0,0,255),2)
# cv2.imshow("compare1",train)
# cv2.waitKey(5000)

# orb = cv2.ORB(nfeatures=500,scaleFactor=1.2,nlevels=15,edgeThreshold=0,WTA_K=2,scoreType=cv2.ORB_HARRIS_SCORE)
#[2,7,8 is wrong

# cv2.imwrite('compare_que.png',imgcopy)
# cv2.imwrite('compare_train.png',train)
# cv2.waitKey(3000)


M,mask = cv2.findHomography(src_pts, dst_pts, 0)

h,w,d = train.shape
pts = np.float32([ [0,0],[w-1,0],[w-1,h-1],[0,h-1] ]).reshape(1,-1,2)
pt1_im = np.float32([w/2,h/2]).reshape(1,-1,2)
pt1_tr = cv2.perspectiveTransform(pt1_im,M)
dst = cv2.perspectiveTransform(pts,M)
cv2.circle(imgcopy,tuple(pt1_tr.reshape(2)),2,(255,255,0),2)
cv2.polylines(imgcopy,np.int32(dst),True,(0,0,255),2,cv2.CV_AA)
cv2.imshow("Window",imgcopy)
cv2.waitKey(3000)

while 1:
    pass
