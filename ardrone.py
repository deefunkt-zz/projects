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
# dark
# blueLower = (53, 21, 16)  # BGR
# blueUpper = (100,51,43)
# light
# blueLower = (210,30,50)  # HSV
# blueUpper = (240, 80, 100)
blueLower = (110,120,70)
blueUpper = (125,220,150)  # BGR of HSV file

centerblue = 0


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

        # subscriber to the navdata, when a message is received call the fn self.GetNavdata
        self.subNavdata = rospy.Subscriber("/ardrone/navdata",dronemsgs.Navdata,self.GetNavdata)  # can it call outside the class?
        self.pubReset = rospy.Publisher("/ardrone/reset",stMsg.Empty,queue_size=3)

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
        cx = -1
        cy = -1
        cvec = np.array([0.,0.])
        width = -1
        height = -1
        theta = -1.0
        time = 0
        dist = 0

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

def seeimage(ros_image,marker):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image,"bgr8") #rgb
    except CvBridgeError as e:
        print(e)
    if me.markercount == 1:
        (rows,cols,channels) = cv_image.shape
        colour = (255,0,0)
        rect = drawmarker(rows,cols,marker)
        cv2.line(cv_image,rect.pt1,rect.pt2,colour)
        cv2.line(cv_image,rect.pt2,rect.pt3,colour)
        cv2.line(cv_image,rect.pt3,rect.pt4,colour)
        cv2.line(cv_image,rect.pt4,rect.pt1,colour)
        cv2.circle(cv_image,(marker.cx*cols/1000,marker.cy*rows/1000),3,colour)
    cv2.imshow("Image window", cv_image)  # cv_image is a matrix

    # cv2.imshow("Image window", cv_image)  # cv_image is a matrix
    # cv2.waitKey(3)
    # mask = cv2.inRange(cv_image, blueLower, blueUpper)
    # cv2.imshow("Masked window", mask)
    # cv2.waitKey(3)
    global centerblue
    [findblock,centerblue] = processimage(cv_image)
    print centerblue
    cv2.imshow("block window", findblock)
    cv2.waitKey(3)

# takes in [x, y] coordinates of marker, and executes a moveForward or moveBack based on error in position
def followImage(marker, height, time, previouspos, previoustime):
    center = np.array([500., 500.])
    kp = 1/1500.
    kd = 0.07
    errorInXPos = float((500 - marker[1]))  # since the errors in camera image and drone reference are inverted, we use
    errorInYPos = float((500 - marker[0]))  # the opposite vector elements in marker[]
    dt = (time - previoustime)
    # errorNow - prevError divided by the time
    derror = ((center - marker) - (center - previouspos)) / dt
    if math.isnan(derror[0]):
        derror = [0., 0.]
    pdControlX = float(kp*errorInXPos + kd*derror[1])
    pdControlY = float(kp*errorInYPos + kd*derror[0])
    print "move in x: " + str(pdControlX) + "\nmove in y: " + str(pdControlY)
    print "Our height is; " + str(height)
    print "The change in Time: " + str(dt)
    p1sec = rospy.Duration(0, 10000000)
    rospy.sleep(p1sec)
    control.moveXY(pdControlX, pdControlY, height)
    return np.array([marker, time])


def processimage(cv_image):
    cv_hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV window", cv_hsv)
    cv2.imwrite('block_hsv.jpg', cv_hsv)
    mask = cv2.inRange(cv_hsv, blueLower, blueUpper)
    # mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cv2.imshow("Masked Window",mask)
    cv2.waitKey(3)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    if len(cnts) > 0:  # only proceed if at least one contour was found
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10: # only proceed if the radius meets a minimum size
            cv2.circle(cv_image, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
    return [cv_image, center]


class twistMatrix:

    def __init__(self, movePublisher):
        self.movePub = movePublisher

    def moveXY(self, x, y, height):
        cmd = Twist()
        cmd.linear.x = 0.25*x
        cmd.linear.y = 0.25*y
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        if height < 150:
            cmd.linear.z = 0.1
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
    image_sub = rospy.Subscriber("/ardrone/image_raw", Image, seeimage, me.marker)
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
    control.moveXY(0,0,0.1) # increase hover height to widen field of view
    rospy.sleep(rospy.Duration(1))

    # while not rospy.is_shutdown():
    previousMarker = np.array([500, 500])
    prevTime = getattr(me.marker, 'time')
    previousMarkerObject = np.array([previousMarker, prevTime])
    while 1:
        try:
            if me.markercount is 1:
                thisfunction = followImage(getattr(me.marker, 'cvec'), getattr(me.marker, 'dist'),getattr(me.marker, 'time'), previousMarkerObject[0], previousMarkerObject[1])
                previousMarkerObject = thisfunction
                # rospy.spin()
            else:
                control.hoverNoAuto()
        except:
            control.land(pubLand)
            this.kill()