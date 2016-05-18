import rospy
import subprocess
import std_msgs.msg as stMsg
import ardrone_autonomy.msg as dronemsgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
# import sys
# import sensor_msgs.msg
from sensor_msgs.msg import Image
# from sensor_msgs.msg import

# Some Constants for GUI
# CONNECTION_CHECK_PERIOD = 250 #ms
# GUI_UPDATE_PERIOD = 20 #ms



def initdrone():
    # roscore = subprocess.Popen('roscore')
    # rospy.sleep(3)
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
    # store.Rotations = [data.rotX, data.rotY, data.rotZ]
    # store.status = data.state
        self.markercount = data.tags_count
    # store.marker_orie = data.tags_orientation
    # store.marker_dis = data.tags_distance
        if self.markercount == 1:
            self.marker.cx = data.tags_xc[0]
            self.marker.cy = data.tags_yc[0]
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

class createrect():
    def __init__(self):
        pt1 = -1
        pt2 = -1
        center = -1




class image_converter():
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,seeimage)
        # self.camerainfo = rospy.Subscriber("/ardrone/camera_info/ardone_front",sensor_msgs.msg.CameraInfo)
        self.cv_image = Image # need to be intialised differently?
    def convert(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8") #rgb
        except CvBridgeError as e:
            print(e)
        # cv2.imshow("Image window",cv_image)
        # cv2.waitKey(3)
        self.cv_image = cv_image

        # store as a tuple? or the output tuple store individual outputs from the bridge?
        #cv_image.shape returns width, height, and number of colours/channels

        # if cols > 60 and rows > 60:   # check is valid image?
            # image,centre,radius,colour
            # cv2.circle(cv_image,(50,50),10,255)

def drawmarker(rows,cols,marker):
    # pt1 top left, pt2 bottom right
    #
        # pt1x = marker.cx*cols/1000 - marker.width*cols/(1000*2)
        # pt1y = marker.cy*rows/1000 + marker.height*rows/(1000*2)
        # pt2x = marker.cx*cols/1000 + marker.width*cols/(1000*2)
        # pt2y = marker.cy*rows/1000 - marker.height*rows/(1000*2)
        # pt1x = marker.cx*rows/1000 - marker.width*rows/(1000*2)
        # pt1y = marker.cy*cols/1000 + marker.height*cols/(1000*2)
        # pt2x = marker.cx*rows/1000 + marker.width*rows/(1000*2)
        # pt2y = marker.cy*cols/1000 - marker.height*cols/(1000*2)
    # pt1x = marker.cx*cols/1000 - marker.width*cols/(1000)
    # pt1y = marker.cy*rows/1000 + marker.height*rows/(1000)
    # pt2x = marker.cx*cols/1000 + marker.width*cols/(1000)
    # pt2y = marker.cy*rows/1000 - marker.height*rows/(1000)
    rect = createrect()
    rect.pt1 = (pt1x,pt1y)
    rect.pt2 = (pt2x,pt2y)
    rect.center = (marker.cx*cols/1000,marker.cy*rows/1000)
    return rect

def seeimage(ros_image,marker):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image,"bgr8") #rgb
    except CvBridgeError as e:
        print(e)

    if me.markercount == 1:
        # (rows,cols,channels) = cv_image.shape
        # rect = drawmarker(rows,cols,marker)
        colour = (255,0,0)
        # cv2.rectangle(cv_image,rect.pt1,rect.pt2,colour,3)
        # cv2.circle(cv_image,rect.center,3,colour)

        cv2.circle(cv_image,(marker.cx*640/1000,marker.cy*360/1000),3,colour)
        cv2.imshow("Image window", cv_image)  # cv_image is a matrix
        cv2.waitKey(3)
    # cv2.waitKey(3) # wait three seconds?


if __name__ == '__main__':
    initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber

    image_sub = rospy.Subscriber("/ardrone/image_raw",Image,seeimage,me.marker)
    # camerainfo = rospy.Subscriber("/ardrone/camera_info/ardone_front",sensor_msgs.msg.CameraInfo)
    # ic = image_converter()

    while 1:
        pass
        # if me.markercount == 1:
        #     seeimage(ic.cv_image, me.marker)

