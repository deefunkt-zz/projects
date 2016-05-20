import rospy
import subprocess
import std_msgs.msg as stMsg
import ardrone_autonomy.msg as dronemsgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
# import sys
# import sensor_msgs.msg
from sensor_msgs.msg import Image
# from sensor_msgs.msg import

# Some Constants for GUI
# CONNECTION_CHECK_PERIOD = 250 #ms
# GUI_UPDATE_PERIOD = 20 #ms

# constants for rectangle drawing
roundel_ratio = 0.639  # assume in center but actually not (long side/total)
roundel_alpha = 0.5051   # in rad = 28.94 degrees (long)
roundel_beta = math.pi - 0.7736  # in rad = 180- 44.32 degrees (short)
r2d = 180/math.pi
d2r = math.pi/180

def initdrone():
    # roscore = subprocess.Popen('roscore')
    # rospy.sleep(3)
    rospy.init_node('dronetest',anonymous=True)
    cv2.destroyAllWindows()

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
    # store.Rotations = [data.rotX, data.rotY, data.rotZ]
    # store.status = data.state
        self.markercount = data.tags_count
    # store.marker_orie = data.tags_orientation
    # store.marker_dis = data.tags_distance
        if self.markercount == 1:
            self.marker.cx = data.tags_xc[0]
            self.marker.cy = data.tags_yc[0]
            self.marker.theta = -data.tags_orientation[0]
            # print(self.marker.theta)
            # print (self.marker.cx ,self.marker.cy)
            self.marker.width = data.tags_width[0]
            self.marker.height = data.tags_height[0]
            # self.marker = {'cx': data.tags_xc[0], 'cy' : data.tags_yc[0], 'width' : data.tags_width[0], 'height' : data.tags_height[0]}
            # return store

class createmarker():
    def __init__(self):
        cx = -1
        cy = -1
        width = -1
        height = -1
        theta = -1.0

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


if __name__ == '__main__':
    initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber

    image_sub = rospy.Subscriber("/ardrone/image_raw",Image,seeimage,me.marker)

    while 1:
        # try:
        pass

        # if me.markercount == 1:
        #     seeimage(ic.cv_image, me.marker)

