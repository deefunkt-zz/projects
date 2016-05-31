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


# abc = 0
# plt.figure(1)
# plt.subplot(211)
# plt.scatter(abc,abc)
# plt.ylim([-500, 500])
# plt.subplot(212)
# plt.scatter(abc+1,abc+1)
# plt.ylim([-500, 500])
# plt.ion()
# plt.show()
# while 1:
#     plt.subplot(211)
#     plt.scatter(abc,abc)
#     plt.subplot(212)
#     plt.scatter(abc*2,abc*2)
#     plt.draw()
#     abc += 1

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
# dark
# blueLower = (53, 21, 16)  # BGR
# blueUpper = (100,51,43)
# light
# blueLower = (210,30,50)  # HSV
# blueUpper = (240, 80, 100)
# blueLower = (110,100,50)
# blueUpper = (125,220,150)  # BGR of HSV file
blueLower = np.array([110,50,50])
blueUpper = np.array([125,225,255])  # BGR of HSV file

# purple
# blueLower = np.array([200,50,20])
# blueUpper = np.array([230,250,250])

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


class BasicDroneController(object):
    def __init__(self):
        self.status = -1
        self.Rotations = -1 # initialise?
        self.markercount = -1
        self.marker_orie = -1
        self.marker_dis = -1
        self.marker = createmarker()    # classes


        # # subscriber to the navdata, when a message is received call the fn self.GetNavdata
        # self.subNavdata = rospy.Subscriber("/ardrone/navdata",dronemsgs.Navdata,self.GetNavdata)  # can it call outside the class?
        # self.pubReset = rospy.Publisher("/ardrone/reset",stMsg.Empty,queue_size=3)

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

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

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
def followImage(marker, width, time,horobpos, horobtime, prevparam, totalintegratedError):

    previouspos = prevparam[0]
    previoustime = prevparam[1]
    prevhorobpos = prevparam[2]
    prevhorobtime = prevparam[3]


    center = np.array([500., 500.])
    kp = 1/8000.
    kd = 0.00075
    ki = 1./2000

    kp_z = 1./500
    kp_yaw = 1./1500
    kd_z = 0.0

    errorInZPos = float(180-horobpos[1])
    errorInYaw = float(320-horobpos[0])
    # print (horobpos[0],horobpos[1])
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
        pidControlX = float(kp*errorInXPos + kd*derror[1] + ki*totalintegratedError[1])
        pidControlY = float(kp*errorInYPos + kd*derror[0] + ki*totalintegratedError[0])
        # pdControlZ = float(kp_z*errorInZPos +kd_z*derrorZ[1])
        pdControlZ = float(kp_z*errorInZPos)
        # pdcontrolYaw = float(kp_yaw*errorInYaw)
        print "move in x: " + str(pidControlX) + "\nmove in y: " + str(pidControlY) + "\nmove in z: " + str(pdControlZ) + "\nyaw: "
        # print "move in z: " + str(pdControlZ)
        # print "Our height is; " + str(height)
        # print "The change in Time: " + str(dt)
        p1sec = rospy.Duration(0, 1000)
        rospy.sleep(p1sec)
        # control.moveXYZyaw(pidControlX, pidControlY, pdControlZ,0, width)
        control.moveXYZ(pidControlX, pidControlY, pdControlZ, width)
        print "\nwidth is: " + str(width)
    return np.array([marker, time,horobpos,horobtime]) # return current values to use as previous values


class analysefront():
    def __init__(self):
        self.cv_image = -1
        self.bridge = CvBridge()
        self.timefront = rospy.get_time()
        self.new = 0
        self.arguments = np.array([320,180,0,0,0,self.timefront])
        self.orb = cv2.ORB()  # Initiate SIFT detector
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)   # create BFMatcher object


    def seeimage(self,ros_image):
        # arguments = objectim.frontcamargs
        self.new = 1
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(ros_image,"bgr8") #rgb
            imgcopy = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
            # cv2.imshow("main",self.cv_image)
            # cv2.waitKey(1)
            # cv2.imwrite('totrain.png',self.cv_image)

            self.timefront = rospy.get_time()

            # find the keypoints and descriptors with SIFT
            kp1, des1 = self.orb.detectAndCompute(Trainimg,None)
            kp2, des2 = self.orb.detectAndCompute(imgcopy,None)

            # Match descriptors.
            matches = self.bf.match(des1,des2)

            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)
            # best 10
            match10 = matches[:10]

            # Apply ratio test
            # good = []
            # for i,m in enumerate(matches):
            #     for j,n in enumerate(matches):
            #         if (i is not j) and (m.distance < 0.75*n.distance):
            #             good.append([m])

            src_pts = np.float32([ kp1[m.queryIdx].pt for m in match10]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in match10]).reshape(-1,1,2)
            M,mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            src_int = totuple(np.int32([src_pts]).reshape(-1,2))
            dstint = np.int32(dst_pts).reshape(-1,2)
            points = totuple(dstint)
            for i in points:
                cv2.circle(imgcopy,i,2,(255,0,0),-1)
            for i in src_int:
                cv2.circle(self.cv_image,i,2,(255,0,0),-1)
            cv2.imshow("compare",imgcopy)
            cv2.waitKey(1)
            cv2.imshow("compare1",self.cv_image)
            cv2.waitKey(1)


            h,w = Trainimg.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # centre = np.float32([ h/2,w/2 ]).reshape(-1,1,2)
            # ptstup = totuple(pts)
            # center = np.float32([h/2,w/2]).reshape(1,1,2)

            dst = cv2.perspectiveTransform(pts,M)
            # dstint = np.int32(dst)
            # edges = dst.reshape(-1,2)
            cv2.polylines(imgcopy,np.int32(dst),True,(0,0,255),3,cv2.CV_AA)
            # centerim = totuple(dst.reshape(2))
            # cv2.circle(imgcopy,centerim,2,(0,0,255),3)

            cv2.imshow("Window",imgcopy)
            cv2.waitKey(3)


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
    setflag = 0
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

    def moveXYZ(self, x, y, z, width):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y

        if (width < 100):
             cmd.linear.z = 0.1
        elif (width > 180):
            cmd.linear.z = -0.1
        else:
            cmd.linear.z = z
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveXYZyaw(self, x, y, z, yaw, width):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        cmd.linear.z = z

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = yaw

        if (width > 150):
             cmd.linear.z = 0.1
        elif (width < 50):
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

if __name__ == '__main__':
    this = initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber
    subNavdata = rospy.Subscriber('/ardrone/navdata', dronemsgs.Navdata, me.GetNavdata)
    frontcam = analysefront()

    Trainimg = cv2.imread('totrain.png',1) # load in colour

    image_sub = rospy.Subscriber("/ardrone/image_raw", Image, frontcam.seeimage)

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

    # stabilize after Launch,
    rospy.sleep(rospy.Duration(1))
    control.hoverNoAuto()
    rospy.sleep(rospy.Duration(3))
    control.moveXYZ(0,0,0.4,50) # increase hover height to widen field of view
    rospy.sleep(rospy.Duration(1))

    # while not rospy.is_shutdown():
    horObPos = np.array([320,180])
    horObTime = rospy.get_time()
    previousMarker = np.array([500, 500])
    prevTime = getattr(me.marker, 'time')
    prevHorTime = rospy.get_time()
    prevHorOb = np.array([320,180])
    previousMarkerObject = np.array([previousMarker, prevTime, prevHorOb, prevHorTime])
    integratedTime = np.zeros(20)
    integratedError = np.zeros([20,2])
    integratorCounter = 0
    print "lets roollll"
    while 1:
        try:
            if frontcam.new is 1:
                # (seeimageargs.frontcamargs,frontcam.new) = processimage(frontcam.cv_image,frontcam.timefront)
                horObPos = frontcam.arguments[0:2]
                horObTime = frontcam.arguments[5]

                # print 'processed image' + str(frontcamargs)

            if me.markercount is 1:
                timeNow = getattr(me.marker, 'time')
                posNow = getattr(me.marker, 'cvec') # [x, y]
                distNow = getattr(me.marker, 'dist')
                if integratorCounter == 20:
                    integratorCounter = 0
                integratedTime[integratorCounter] = timeNow - previousMarkerObject[1] # the dt
                integratedError[integratorCounter] = posNow - previousMarkerObject[0] # the dr, [dx, dy]
                integratorCounter += 1
                totalIntegratedError =np.dot(integratedTime, integratedError)
                thisfunction = followImage(posNow, distNow,timeNow,
                                           horObPos, horObTime, previousMarkerObject, totalIntegratedError)
                # mycounter += 1
                previousMarkerObject = thisfunction
                # rospy.spin()
            else:
                control.hoverNoAuto()
        except Exception as e:
             print e
            # control.land(pubLand)
            # this.kill()