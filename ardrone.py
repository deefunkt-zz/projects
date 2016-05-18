#!/bin/env python
import rospy
import subprocess
import std_msgs.msg as stMsg
import ardrone_autonomy.msg as dronemsgs
import cv2
import shlex
from cv_bridge import CvBridge, CvBridgeError
# import sys
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata


# todo. capture a few seconds worth of Navdata for later analysis
# todo. track marker over multiple images
# todo. move ardrone via python and GeometryTwist Messages
# todo. implement control system to track marker in middle of camera image
# todo.
# something to to with assigning resources between GUI and ROS topics
# from threading import Lock

# Some Constants
# CONNECTION_CHECK_PERIOD = 250 #ms
# GUI_UPDATE_PERIOD = 20 #ms
# DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


def initdrone():
    try:
        launchcommand = shlex.split('roslaunch ardrone_autonomy ardrone.launch')
        roscore = subprocess.Popen(launchcommand)
        # rospy.sleep(3)
        # ardronedriver = subprocess.Popen('rosrun ardrone_autonomy ardrone_driver')

        rospy.sleep(10.)
    except ValueError:
        print('invalid')

    rospy.init_node('dronetest',anonymous=True)


class BasicDroneController:
    def __init__(self):
        self.status = -1
        self.Rotations = -1 # initialise?
        self.markercount = -1
        self.marker_orie = -1
        self.marker_dis = -1

        # subscriber to the navdata, when a message is received call the fn self.GetNavdata
        # self.subNavdata = rospy.Subscriber("/ardrone/navdata",dronemsgs.Navdata,self.GetNavdata)
        # self.pubReset = rospy.Publisher("/ardrone/reset",stMsg.Empty,queue_size=3)
        # self.pubTakeoff = rospy.Publisher("/ardrone/takeoff",stMsg.Empty,queue_size=5)
        # self.pubLand = rospy.Publisher("/ardrone/land",stMsg.Empty,queue_size=5)


    # retrive and store data
    def GetNavdata(self,data):
        self.Rotations = [data.rotX, data.rotY, data.rotZ]
        self.status = data.state
        if data.tags_count is 1:
            self.markercount = data.tags_count
            self.marker_orie = data.tags_orientation
            self.taglocation = [data.tags_xc[0], data.tags_yc[0]]
            self.marker_dis = data.tags_distance #in cm
            print('[xc, yc] = ' + str(self.taglocation) + '\n')



class image_converter:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.seeimage)



    def seeimage(self,data):
        try:
            # calling inside cvbridge
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8") #rgb?
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window",cv_image)  # cv_image is a matrix
        cv2.waitKey(3) # wait three seconds?

        try:
            # the publisher node
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber
    ic = image_converter()
    subNavdata = rospy.Subscriber("/ardrone/navdata", dronemsgs.Navdata, me.GetNavdata)
    image_sub = rospy.Subscriber("/ardrone/image_raw", Image, ic.seeimage)
    pubReset = rospy.Publisher("/ardrone/reset", stMsg.Empty,queue_size=10)
    pubTakeoff = rospy.Publisher("/ardrone/takeoff", stMsg.Empty,queue_size=10)
    pubLand = rospy.Publisher("/ardrone/land", stMsg.Empty,queue_size=10)
    five_seconds = rospy.Duration(5)
    rospy.sleep(five_seconds)
    pubTakeoff.publish(stMsg.Empty)
    rospy.sleep(five_seconds)
    pubLand.publish(stMsg.Empty())

    while not rospy.is_shutdown():
        rospy.spin()