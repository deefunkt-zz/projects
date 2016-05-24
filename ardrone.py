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
# import sys
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata


# todo. capture a few seconds worth of Navdata for later analysis
# todo. track marker over multiple images
# todo. move ardrone via python and GeometryTwist Messages
# todo. implement control system to track marker in middle of camera image
# todo.

# constants for rectangle drawing
roundel_ratio = 0.639  # assume in center but actually not (long side/total)
roundel_alpha = 0.5051   # in rad = 28.94 degrees (long)
roundel_beta = math.pi - 0.7736  # in rad = 180- 44.32 degrees (short)
r2d = 180/math.pi
d2r = math.pi/180


def initdrone():
    try:
        launchcommand = shlex.split('roslaunch ardrone_autonomy ardrone.launch')
        roscore = subprocess.Popen(launchcommand)
        # rospy.sleep(3)
        # ardronedriver = subprocess.Popen('rosrun ardrone_autonomy ardrone_driver')

        rospy.sleep(10.)
    except RuntimeError:
        print('invalid launch')

    rospy.init_node('dronetest',anonymous=True)


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
        self.markercount = data.tags_count
        if self.markercount == 1:
            self.marker.cx = data.tags_xc[0]
            self.marker.cy = data.tags_yc[0]
            self.marker.cvec = np.array([self.marker.cx, self.marker.cy])
            self.marker.theta = -data.tags_orientation[0]
            # print(self.marker.theta)
            # print (self.marker.cx ,self.marker.cy)
            self.marker.width = data.tags_width[0]
            self.marker.height = data.tags_height[0]
            self.marker.time = data.tm
            # self.marker = {'cx': data.tags_xc[0], 'cy' : data.tags_yc[0], 'width' : data.tags_width[0], 'height' : data.tags_height[0]}
            # return store

class createmarker():
    def __init__(self):
        cx = -1
        cy = -1
        cvec = np.array([0.,0.])
        width = -1
        height = -1
        theta = -1.0
        time = 0

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
    # width = marker.width*cols/(1000) # not valid for orientated
    # height= marker.height*rows/(1000)
    # pt1 top right, pt2 top left, pt3 bottom left, pt4 bottom right

    # pt1x = marker.cx*cols/1000 - width
    # pt1y = marker.cy*rows/1000 + height
    #
    # pt2x = marker.cx*cols/1000 + width
    # pt2y = marker.cy*rows/1000 - height

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
        # rect = drawmarker(rows,cols,marker)
        colour = (255,0,0)
        # cv2.rectangle(cv_image,rect.pt1,rect.pt2,colour,3)
        # cv2.circle(cv_image,rect.center,3,colour)
        ################ EDITED Y VALUE
        # if marker.cy < 180:
        #     cyedit = (180-marker.cy)*180/135
        #     cyedit = cyedit-135
        # else:
        #     cyedit = (marker.cy-180)*180/135
        #     cyedit = cyedit + 135
        #
        # marker.cy = cyedit
        rect = drawmarker(rows,cols,marker)
        # not rotated
        # cv2.rectangle(cv_image,rect.pt1,rect.pt2,colour,3)
        # rotated rectangle
        cv2.line(cv_image,rect.pt1,rect.pt2,colour)
        cv2.line(cv_image,rect.pt2,rect.pt3,colour)
        cv2.line(cv_image,rect.pt3,rect.pt4,colour)
        cv2.line(cv_image,rect.pt4,rect.pt1,colour)


        cv2.circle(cv_image,(marker.cx*cols/1000,marker.cy*rows/1000),3,colour)

    cv2.imshow("Image window", cv_image)  # cv_image is a matrix
    cv2.waitKey(3)
        # cv2.destroyAllWindows()
        # k = cv2.waitKey(3)  & 0xFF   # press a key in that time to keep it? (64 bit machine?)
        # if k == 27:         # wait for ESC key to exit
        #     cv2.destroyAllWindows()
        # else: # save and destroy
        #     cv2.imwrite('TestingRoundel.png',cv_image)
        #     cv2.destroyAllWindows()
    # cv2.waitKey(3) # wait three seconds?


def Pcontrol(setpos,cpos,ctime,ppos,ptime):
    vmaxP = 0.08
    vmaxD = 0.05
    maxdev = np.array([500, 500])
    dt = (ctime - ptime)/1000000
    derror = ((setpos - cpos) - (setpos - ppos))/dt
    # error = desired - current
    # output = -K*error
    output = -vmaxP*(setpos-cpos)/maxdev + vmaxD*derror/maxdev
    return output


class twistMatrix:

    def __init__(self, movePublisher):
        self.movePub = movePublisher

    def moveForward(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0.2
        cmd.linear.y = 0.1

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveBack(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = -0.25
        cmd.linear.y = 0.05

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
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0.2

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def moveRight(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = -0.2

        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def hover(self):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def xyinput(self, vec):
        cmd = Twist()
        cmd.linear.z = 0
        cmd.linear.x = -vec[1]
        cmd.linear.y = -vec[0]-(0.4*vec[1])
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.movePub.publish(cmd)

    def land(self, landPub):
        self.hover()
        rospy.sleep(rospy.Duration(0.5))
        landPub.publish(EmptyMessage)


if __name__ == '__main__':
    initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber
    subNavdata = rospy.Subscriber('/ardrone/navdata', dronemsgs.Navdata, me.GetNavdata)
    image_sub = rospy.Subscriber("/ardrone/image_raw", Image, seeimage, me.marker)
    pubReset = rospy.Publisher("/ardrone/reset", stMsg.Empty, queue_size=1)
    pubTakeoff = rospy.Publisher("/ardrone/takeoff", stMsg.Empty, queue_size=1)
    pubLand = rospy.Publisher("/ardrone/land", stMsg.Empty,queue_size=1)
    pubCmd = rospy.Publisher("/cmd_vel", Twist, queue_size = 6)
    EmptyMessage = stMsg.Empty()
    prevMarker = createmarker()
    currentMarker = createmarker()
    five_seconds = rospy.Duration(5)
    two_seconds = rospy.Duration(2)



##### control sequence
    rospy.sleep(five_seconds)
    pubTakeoff.publish(EmptyMessage)
    rospy.sleep(two_seconds)
    control = twistMatrix(pubCmd)
    #control.moveForward()
    # rospy.sleep(rospy.Duration(2))
    # control.hover()
    # rospy.sleep(rospy.Duration(1))
    # control.moveBack()
    # rospy.sleep(rospy.Duration(2))
    # control.land(pubLand)
    loop = 1
    randomFlag = False

    while not rospy.is_shutdown() :
        if loop < 200 and me.markercount==1:

            if randomFlag is False:
                #prevMarker = getattr(me.marker, 'time') #
                prevtime = getattr(me.marker, 'time')  #
                prevpos = getattr(me.marker,'cvec')
                randomFlag = True
            else:
                #currentMarker = getattr(me.marker, 'time') #
                currtime = getattr(me.marker, 'time')  #
                currpos = getattr(me.marker, 'cvec')
                vel = Pcontrol(np.array([500,500]), currpos,currtime,prevpos,prevtime)
                prevtime = currtime
                prevpos = currpos
                print (-vel[1],-vel[0])
                control.xyinput(vel)
            rospy.sleep(rospy.Duration(0.1)) # wait a bit
            loop += 1
        elif loop < 200:
            control.hover()
            rospy.sleep(rospy.Duration(0.1)) # wait a bit
            loop += 1
        else:
            control.land(pubLand)
