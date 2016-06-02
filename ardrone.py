#!/bin/env python
import rospy
import subprocess
import std_msgs.msg as stMsg
from geometry_msgs.msg import Twist
import ardrone_autonomy.msg as dronemsgs
import cv2
import shlex
import math
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from ardrone_autonomy.msg import Navdata



#
# done. track marker over multiple images
# done. move ardrone via python and GeometryTwist Messages
# done. recieve all telemetry using appropriate publisher/subscriber pairs
# todo. implement control system to track marker in middle of camera image
# SUB   todo. investigate the reference frames of the quadcotper thoroughly
# SUB   todo. fix autohovering behaviour which causes drone to interpret moving marker as a moving ground surface
# todo. capture a few seconds worth of Navdata for later analysis

# constants for rectangle drawing
roundel_ratio = 0.639  # assume in center but actually not (long side/total)
roundel_alpha = 0.5051   # in rad = 28.94 degrees (long)
roundel_beta = math.pi - 0.7736  # in rad = 180- 44.32 degrees (short)
r2d = 180/math.pi
d2r = math.pi/180
blueLower = np.array([110,50,50])
blueUpper = np.array([125,225,255])  # BGR of HSV file

# plt.figure(1)
# plt.subplot(211)
# plt.subplot(212)
# plt.ion()
# plt.show()


def initdrone():
    try:
        launchcommand = shlex.split('roslaunch ardrone_autonomy ardrone.launch')
        roscore = subprocess.Popen(launchcommand)
        rospy.sleep(10.)
        rospy.init_node('dronetest', anonymous=True)
        return roscore
    except RuntimeError:
        print('invalid launch')

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

class BasicDroneController(object):
    def __init__(self):
        self.status = -1
        self.Rotations = -1 # initialise?
        self.markercount = -1
        self.marker_orie = -1
        self.marker_dis = -1
        self.marker = createmarker()    # classes


    # retrive and store data??
    def GetNavdata(self,data):
        self.marker.time = data.tm
        self.markercount = data.tags_count
        if self.markercount == 1:
            self.marker.cx = data.tags_xc[0]
            self.marker.cy = data.tags_yc[0]
            self.marker.cvec = np.array([self.marker.cx, self.marker.cy])
            self.marker.theta = -data.tags_orientation[0]
            self.marker.width = data.tags_width[0]
            self.marker.height = data.tags_height[0]
            self.marker.dist = data.tags_distance[0]


class createmarker():
    def __init__(self):
        self.cx = -1
        self.cy = -1
        self.cvec = np.array([0.,0.])
        self.width = -1
        self.height = -1
        self.theta = -1.0
        self.time = 0
        self.dist = 0

class createrect():
    def __init__(self):
        pt1 = -1
        pt2 = -1
        pt3 = -1
        pt4 = -1
        center = -1


def drawmarker(rows,cols,marker):
    # assume height and width doesnt need to /1000* cols||rows
    Hlong = math.sqrt((marker.width*roundel_ratio)**2 + (float(marker.height)/2)**2)
    Hshort = math.sqrt((marker.width*(1-roundel_ratio))**2+(float(marker.height)/2)**2)
    pt1x = (Hlong*math.cos(marker.theta*d2r+roundel_alpha)) + (float(marker.cx)/1000)*cols
    pt1y = (Hlong*math.sin(marker.theta*d2r+roundel_alpha)) + (float(marker.cy)/1000)*rows
    pt2x = (Hshort*math.cos(marker.theta*d2r+roundel_beta)) + (float(marker.cx)/1000)*cols
    pt2y = (Hshort*math.sin(marker.theta*d2r+roundel_beta)) + (float(marker.cy)/1000)*rows
    pt3x = (Hshort*math.cos(marker.theta*d2r-roundel_beta)) + (float(marker.cx)/1000)*cols
    pt3y = (Hshort*math.sin(marker.theta*d2r-roundel_beta)) + (float(marker.cy)/1000)*rows
    pt4x = (Hlong*math.cos(marker.theta*d2r-roundel_alpha)) + (float(marker.cx)/1000)*cols
    pt4y = (Hlong*math.sin(marker.theta*d2r-roundel_alpha)) + (float(marker.cy)/1000)*rows
    rect = createrect()
    rect.pt1 = (int(pt1x),int(pt1y))
    rect.pt2 = (int(pt2x),int(pt2y))
    rect.pt3 = (int(pt3x),int(pt3y))
    rect.pt4 = (int(pt4x),int(pt4y))
    rect.center = (marker.cx*cols/1000,marker.cy*rows/1000)
    return rect


# takes in [x, y] coordinates of marker, and executes a moveForward or moveBack based on error in position
def followImage(marker, height, time,horobpos, horobtime, prevparam, integratedError):
    previouspos = prevparam[0]
    previoustime = prevparam[1]
    prevhorobpos = prevparam[2]
    prevhorobtime = prevparam[3]


    center = np.array([500., 500.])
    kpx = 1/16000.
    kpy = 1/9000.
    kdx = 0.00045
    kdy = 0.00080
    # kd = 0
    kix = 1/48000000000. ## should be 10-6 * length of time over which error is accumulated (1/ (6 x 10_9))
    kiy = 1/27000000000.
    pidControlrotZ = 0.
    kp_z = 1./500
    kd_z = 0.0
    kp_Rot = 1./500
    pidControlX = 0.
    pidControlY = 0.
    errorInZPos = float(180-horobpos[1])
    print str(horobpos[1])
    # errorInZRot = float(320 - horobpos[0])
    errorInXPos = float((500 - marker[1]))  # since the errors in camera image and drone reference are inverted, we use
    errorInYPos = float((500 - marker[0]))  # the opposite vector elements in marker[]

    # plt.figure(1)
    # plt.subplot(211)
    # plt.scatter(counter,errorInXPos)
    # plt.ylim([-500, 500])
    # plt.subplot(212)
    # plt.scatter(counter, errorInYPos)
    # plt.ylim([-500, 500])
    # plt.draw()

    # dtZ = (horobtime - prevhorobtime)
    dt = (time - previoustime)
    if dt > 0:
        # errorNow - prevError divided by the time
        derror = (previouspos - marker)/dt
        # derrorZ = (prevhorobpos-horobpos)/dtZ

        if math.isnan(derror[0]):
            derror = [0., 0.]
    # if math.isnan(derrorZ[0]):
    #     derrorZ =0
        pidControlX = float(kpx*errorInXPos + kdx*derror[1] + kix*integratedError[1])
        if (abs(errorInXPos) <= 50):
            pidControlX = 0.
        pidControlY = float(kpy*errorInYPos + kdy*derror[0] + kiy*integratedError[0])
        if (abs(errorInYPos) <= 50):
            pidControlY = 0.
        # pdControlZ = float(kp_z*errorInZPos +kd_z*derrorZ[1])
        pdControlZ = float(kp_z*errorInZPos)
        if (abs(pdControlZ) > 0.01):
            print pdControlZ
        # pidControlrotZ = float(kp_Rot*errorInZRot)
        # print "Our height is; " + str(height)
        # print "The change in Time: " + str(dt)
        # if ((errorInYPos < 50) and (errorInXPos < 50) and (pidControlX < 0.02) and (pidControlY < 0.02)):     #used to test rotation while hovering over roundel
        #     rotZ = 0.3
        #     print "rotate"
        control.moveXYZ(pidControlX, pidControlY, pdControlZ,pidControlrotZ,height)
        p1sec = rospy.Duration(0, 1000000)
        rospy.sleep(p1sec)
        #print "X: " + str(pidControlX) + "         Y:  " + str(pidControlY)
        # if (pidControlX > 0.1 or pidControlY > 0.1):
            # print "XK: " + str(kp*errorInXPos) + " XD: " + str(kd*derror[1]) +" XI: " + str(ki*integratedError[1])
            # print "YK: " + str(kp * errorInYPos) + " YD: " + str(kd * derror[0]) + " YI: " + str(ki * integratedError[0])
    return np.array([marker, time,horobpos,horobtime,pidControlX,pidControlY]) # return current values to use as previous values


class analysefront():
    def __init__(self):
        self.cv_image = -1
        self.bridge = CvBridge()
        self.timefront = rospy.get_time()
        self.index = 0
        self.arguments = np.array([320,180,0,0,0,self.timefront])
        self.orb = cv2.ORB(nfeatures=200,scaleFactor=1.2,nlevels=20,edgeThreshold=0,WTA_K=2,patchSize=31)
        # self.orb = cv2.ORB()  # Initiate SIFT detector
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)   # create BFMatcher object
        # FLANN_INDEX_LSH = 6
        # index_params= dict(algorithm = FLANN_INDEX_LSH,
        #            table_number = 12, # 12
        #            key_size = 12,     # 20
        #            multi_probe_level = 1) #2
        # search_params = dict(checks=200)
        # self.flann = cv2.FlannBasedMatcher(index_params,search_params)
        # self.train = cv2.imread('/home/astrochick/Documents/projects/ardrone/Pictures/Train_check.jpg',1)          # queryImage
        self.train = cv2.imread('/home/deefunkt/Storage/ROS/projects/Train_mandalas1.jpg',1)          # queryImage
        self.kp1, self.des1 = self.orb.detectAndCompute(self.train,None)
        self.orb = cv2.ORB(nfeatures=1000,scaleFactor=1.2,nlevels=20,edgeThreshold=0,WTA_K=2,patchSize=31)


    def seeimage(self,ros_image,img_count):
        # arguments = objectim.frontcamargs


        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image,"bgr8") #rgb
            # imgcopy = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
            # cv2.imshow("main",self.cv_image)
            # cv2.waitKey(1)
            # cv2.imwrite('Train_mandalas1.jpg',self.cv_image)
            img_count.index += 1
            if img_count.index >= 10:
                img_count.index = 0
                self.timefront = rospy.get_time()
                img_count.time = self.timefront
                # find the keypoints and descriptors with SIFT
                kp2, des2 = self.orb.detectAndCompute(self.cv_image,None)
                if des2.__len__()>5:

                # Match descriptors.
                    matches = self.bf.match(self.des1,des2)
                    # matches = self.flann.knnMatch(self.des1,des2,k=1)
                    matches = sorted(matches, key=lambda x:x.distance)
                    good = matches[:15]
                    # good = []
                # for m_n in matches:
                #     if len(m_n) != 2:
                #         continue
                #     (m,n) = m_n
                #     if m.distance < 0.75*n.distance:
                #         good.append(m)
                #         good.append(n)

                    if good.__len__() >= 15:
                        # src_pts = np.float32([self.kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                # src_int = totuple(np.int32([src_pts]).reshape(-1,2))
                # dst_int = totuple(np.int32([dst_pts]).reshape(-1,2))
                # for i in dst_int:
                #     cv2.circle(imgcopy,i,2,(255,0,0),-1)
                #     # cv2.imshow("compare",imgcopy)
                #     # cv2.waitKey(1000)
                # for i in src_int:
                #     cv2.circle(self.train,i,2,(255,0,0),-1)
                #     # cv2.imshow("compare1",train)
                #     # cv2.waitKey(1000)
                # cv2.imshow("compare",self.train)
                # cv2.waitKey(1)
                # # cv2.imshow("compare1",train)


                # src_int = totuple(np.int32([src_pts]).reshape(-1,2))
                # dst_int = totuple(np.int32([dst_pts]).reshape(-1,2))
                # for i in dst_int:
                #     cv2.circle(imgcopy,i,2,(255,0,0),-1)
                #     # cv2.imshow("compare",imgcopy)
                #     # cv2.waitKey(1000)
                # for i in src_int:
                #     cv2.circle(self.train,i,2,(255,0,0),-1)
                #     # cv2.imshow("compare1",train)
                #     # cv2.waitKey(1000)
                # cv2.imshow("compare",self.train)
                # cv2.waitKey(1)
                # # cv2.imshow("compare1",train)

            # src_pts = np.float32([ kp1[m.queryIdx].pt for m in match10]).reshape(-1,1,2)
            # dst_pts = np.float32([ kp2[m.trainIdx].pt for m in match10]).reshape(-1,1,2)
            # M,mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            # matchesMask = mask.ravel().tolist()
            #
            # src_int = totuple(np.int32([src_pts]).reshape(-1,2))
            # dstint = np.int32(dst_pts).reshape(-1,2)
            # points = totuple(dstint)
            # for i in points:
            #     cv2.circle(imgcopy,i,2,(255,0,0),-1)
            # for i in src_int:
            #     cv2.circle(self.cv_image,i,2,(255,0,0),-1)
            # cv2.imshow("compare",imgcopy)
            # cv2.waitKey(1)
            # cv2.imshow("compare1",self.cv_image)
            # cv2.waitKey(1)

                        size = self.cv_image.shape
                        bareim = np.zeros(size[:2],np.uint8)
                        index = totuple(np.int32(dst_pts).reshape(-1,2))
                        for i in index:
                            cv2.circle(bareim,i,7,(255),10)
                        cv2.imshow("mask", bareim)
                        cv2.waitKey(1)

                        dst_vec = dst_pts.reshape(-1,2)
                        (devx,devy) = np.std(dst_vec,0)

                        # print (devx,devy)
                        if devx < 130 and devy < 70:
                            cnts = cv2.findContours(bareim.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
                            if len(cnts) > 0:  # only proceed if at least one contour was found
                                c = max(cnts, key=cv2.contourArea)
                                ((x, y), radius) = cv2.minEnclosingCircle(c)
                                if radius > 12:
                                    M = cv2.moments(c)
                                    img_count.center[0] = int(M["m10"] / M["m00"])
                                    img_count.center[1] = int(M["m01"] / M["m00"])
                                    cv2.circle(self.cv_image,tuple(img_count.center),3,(255,0,0),3)
                                else:
                                    img_count.center = np.array([320,180])
                        else:
                            img_count.center = np.array([320, 180])
                    else:
                        img_count.center = np.array([320, 180])

                else:
                    img_count.center = np.array([320, 180])


            cv2.circle(self.cv_image, tuple(img_count.center), 3, (255, 0, 0), 3)
            cv2.imshow("Window", self.cv_image)
            cv2.waitKey(1)
                # cv2.waitKey(3)
                # pass
                # cv2.imshow("Window",self.cv_image)
                # cv2.waitKey(1)
            # h,w = Trainimg.shape
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # centre = np.float32([ h/2,w/2 ]).reshape(-1,1,2)
            # ptstup = totuple(pts)
            # center = np.float32([h/2,w/2]).reshape(1,1,2)

            # dst = cv2.perspectiveTransform(pts,M)
            # dstint = np.int32(dst)
            # edges = dst.reshape(-1,2)
            # cv2.polylines(imgcopy,np.int32(dst),True,(0,0,255),3,cv2.CV_AA)
            # centerim = totuple(dst.reshape(2))
            # cv2.circle(imgcopy,centerim,2,(0,0,255),3)




            # Draw first 10 matches.
            # img3 = cv2.drawMatches(Trainimg,kp1,self.cv_image,kp2,matches[:10], flags=2)

            # plt.imshow(img3),plt.show()
            # (self.arguments,dis_image,self.new) = processimage(self.cv_image,self.timefront)
            # cv2.imshow("Image Window",dis_image)
            # cv2.imshow("Image Window",self.cv_image)

            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
        # if me.markercount == 1:  # for bottom camera
        #     (rows,cols,channels) = cv_image.shape
        #     colour = (255,0,0)
        #     rect = drawmarker(rows,cols,marker)
        #     cv2.line(cv_image,rect.pt1,rect.pt2,colour)
        #     cv2.line(cv_image,rect.pt2,rect.pt3,colour)
        #     cv2.line(cv_image,rect.pt3,rect.pt4,colour)
        #     cv2.line(cv_image,rect.pt4,rect.pt1,colour)
        #     cv2.circle(cv_image,(marker.cx*cols/1000,marker.cy*rows/1000),3,colour)


def processimage(cv_image,time):
    cv_hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV window", cv_hsv)
    # cv2.imwrite('block_hsv.jpg', cv_hsv)
    mask = cv2.inRange(cv_hsv, blueLower, blueUpper)
    # cv2.imshow("Masked Window",mask)
    # cv2.waitKey(3)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    frontcamargs = np.array([320,180,0,0,0,time])
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:  # only proceed if at least one contour was found
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        frontcamargs[2] = int(radius)
        frontcamargs[3] = int(x)
        frontcamargs[4] = int(y)
        M = cv2.moments(c)
        frontcamargs[0] = int(M["m10"] / M["m00"])
        frontcamargs[1] = int(M["m01"] / M["m00"])

        if frontcamargs[2] > 5: # only proceed if the radius meets a minimum size
            cv2.circle(cv_image, (int(frontcamargs[0]),int(frontcamargs[1])), 5, (0, 0, 255), -1)
            cv2.circle(cv_image, (int(frontcamargs[3]),int(frontcamargs[4])), int(frontcamargs[2]),(0, 255, 255), 2)
            # cv2.imshow("Image window", cv_image)
            # cv2.waitKey(1)
            # frontcamargs = np.array(320,180,0,0,0,time)
        #     radius = -1
        #     center = np.array([-1,-1]) # didnt see
        # center = np.array([-1, -1])
        # radius = -1
        # circle = np.array([-1, -1])
    # return (center, radius, circle,time)
    # return (frontcamargs,setflag)
    return (frontcamargs,cv_image,setflag)


class classargs():
    def __init__(self):
        self.frontcamargs = np.array([320,180,0,0,0,0])


class twistMatrix:

    def __init__(self, movePublisher):
        self.movePub = movePublisher

    def moveXYZ(self, x, y, z, rotation, height):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        cmd.linear.z = z

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = rotation

        if (height < 140):
             cmd.linear.z = 0.25
        elif (height > 200):
            cmd.linear.z = -0.2
        self.movePub.publish(cmd)

    def autoHover(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def hoverNoAuto(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.angular.x = 0.1
        cmd.angular.y = 0.2
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveForward(self):
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.linear.y = 0
        cmd.linear.z = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveBack(self):
        cmd = Twist()
        cmd.linear.x = -0.3
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveUp(self):
        cmd = Twist()
        cmd.linear.z = 0.3
        cmd.linear.x = 0
        cmd.linear.y = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveDown(self):
        cmd = Twist()
        cmd.linear.z = -0.3
        cmd.linear.x = 0
        cmd.linear.y = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def rotateCCW(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0.3
        self.movePub.publish(cmd)

    def rotateCW(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = -0.3
        self.movePub.publish(cmd)

    def moveLeft(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0.2
        cmd.linear.z = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveRight(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = -0.2
        cmd.linear.z = 0

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def land(self, landPub):
        self.autoHover()
        rospy.sleep(rospy.Duration(0.5))
        landPub.publish(EmptyMessage)

    def takeoff(self, takeoffPub):
        rospy.sleep(rospy.Duration(2))
        takeoffPub.publish(EmptyMessage)

    def transform2DroneRef(self, vectorXY):
        vectorXYdrone= (1/(math.sqrt(2)))*np.dot(np.array([[1, -1], [-1, -1]]), np.transpose(vectorXY))
        print "vectorXYdrone is :" + str(vectorXYdrone)
        return vectorXYdrone

class indexclass():
    def __init__(self):
        self.index = 0
        self.center = np.array([500,500])
        self.time = 0

if __name__ == '__main__':
    this = initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber
    subNavdata = rospy.Subscriber('/ardrone/navdata', dronemsgs.Navdata, me.GetNavdata)
    frontcam = analysefront()
    # seeimageargs = classargs()
    # centerblue = np.array([-1,-1])
    # radiusblue = -1
    # circle = np.array([-1,-1])
    img_count = indexclass()
    image_sub = rospy.Subscriber("/ardrone/image_raw", Image, frontcam.seeimage,img_count)

    # image_sub = rospy.Subscriber("/ardrone/image_raw", Image, seeimage, me.marker)

    pubReset = rospy.Publisher("/ardrone/reset", stMsg.Empty, queue_size=1)
    pubTakeoff = rospy.Publisher("/ardrone/takeoff", stMsg.Empty, queue_size=1)
    pubLand = rospy.Publisher("/ardrone/land", stMsg.Empty, queue_size=1)
    pubCmd = rospy.Publisher("/cmd_vel", Twist, queue_size=6)
    EmptyMessage = stMsg.Empty()
    five_seconds = rospy.Duration(5)
    two_seconds = rospy.Duration(2)

##### takeoff control sequence
    rospy.sleep(two_seconds)
    # pubTakeoff.publish(EmptyMessage)
    control = twistMatrix(pubCmd)

    # # stabilize after Launch,
    # rospy.sleep(rospy.Duration(1))
    # control.hoverNoAuto()
    # rospy.sleep(rospy.Duration(3))
    # control.moveXYZ(0,0,0.1,0,160) # increase hover height to widen field of view
    # rospy.sleep(rospy.Duration(1))

    # while not rospy.is_shutdown():
    horObPos = np.array([320,180])
    horObTime = rospy.get_time()
    previousMarker = np.array([500, 500])
    posNow = previousMarker
    prevTime = getattr(me.marker, 'time')
    timeNow = prevTime
    distNow = 150.
    prevHorTime = rospy.get_time()
    prevHorOb = np.array([320,180])
    previousMarkerObject = np.array([previousMarker, prevTime, prevHorOb, prevHorTime,0.,0.])
    thisfunction = previousMarkerObject
    integratedTime = np.zeros(100)
    integratedError = np.zeros([100,2])
    integratorCounter = 0
    myTime = rospy.get_time()
    # errorFile = open('logErrorskdxy.txt', 'a')
    # timeFile = open('logTimesdkdxy.txt', 'a')
    print "GET TO THE CHOPPAAAA!!!!"
    while 1:
        sinceMarker = (rospy.get_time() - myTime)
        try:
            # img_count.index += 1
            if img_count.index >= 9:
                # img_count.index = 0
                # seeimageargs.frontcamargs = processimage(frontcam.cv_image,frontcam.timefront)
                # print horObPos
                horObTime = img_count.time
            if (me.markercount == 1):
                horObPos = img_count.center
                timeNowcheck = getattr(me.marker,'time')
                if (timeNowcheck != timeNow):
                    timeNow = getattr(me.marker, 'time')
                    myTime = rospy.get_time()
                    posNow = getattr(me.marker, 'cvec') # [x, y]
                    if (425 < posNow[0] < 575):
                        posNow[0] = 500
                    if (425 < posNow[1] < 575):
                        posNow[1] = 500
                    distNow = getattr(me.marker, 'dist')
                    if integratorCounter == 100:
                        integratorCounter = 0
                    integratedTime[integratorCounter] = timeNow - previousMarkerObject[1] # the dt
                    integratedError[integratorCounter] = np.array([500, 500]) - posNow # the dr, [dx, dy]
                    # errorFile.write(str(integratedError[integratorCounter]) + '\n')
                    # timeFile.write(str(myTime) + '\n')
                    integratorCounter += 1
                    totalIntegratedError =np.dot(integratedTime, integratedError)
                    thisfunction = followImage(posNow, distNow,timeNow,
                                               horObPos, horObTime, previousMarkerObject, totalIntegratedError)
                    previousMarkerObject = thisfunction

            #IF the drone loses the marker, keep the previous movement command for 1 sec in attempt
            # to re-acquire. if 1 sec passed, skip this block and hover
            elif (sinceMarker < math.pow(10, 1)):
                integratedTime = np.zeros(100)
                integratedError = np.zeros([100, 2])
                integratorCounter = 0
                cmd = Twist()
                cmd.linear.z = 0
                cmd.linear.x = previousMarkerObject[4]
                cmd.linear.y = previousMarkerObject[5]
                cmd.angular.x = 0
                cmd.angular.y = 0
                cmd.angular.z = 0
                #print " Marker Lost.........X: " + str(previousMarkerObject[4]) + "      Y: " + str(previousMarkerObject[5])
                pubCmd.publish(cmd)
                #rospy.sleep(rospy.Duration(1))
            # elif sinceMarker > 20:
            #      errorFile.close()
            #      timeFile.close()

            else:
                control.hoverNoAuto()
        except Exception as e:
            print e
            control.land(pubLand)
            print "landing"
            # this.kill()
