import rospy
import std_msgs.msg as stMsg
import ardrone_autonomy.msg as dronemsgs
import cv2
from cv_bridge import CvBridge, CvBridgeError
# import sys
from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
# from drone_status import DroneStatus

# something to to with assigning resources between GUI and ROS topics
# from threading import Lock

# Some Constants
# CONNECTION_CHECK_PERIOD = 250 #ms
# GUI_UPDATE_PERIOD = 20 #ms
# DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


def initdrone():
    rospy.init_node('dronetest',anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    cv2.destroyAllWindows()

    # rospy.Subscriber("/ardrone/navdata",dronemsgs.Navdata,callback)
    #rospy.spin()

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s",data.data)

class BasicDroneController(object):
    def __init__(self):
        self.status = -1
        self.Rotations = -1 # initialise?

        # subscriber to the navdata, when a message is received call the fn self.GetNavdata
        self.subNavdata = rospy.Subscriber("/ardrone/navdata",dronemsgs.Navdata,self.GetNavdata)
        self.pubReset = rospy.Publisher("/ardrone/reset",stMsg.Empty,queue_size=3)

    # retrive and store data
    def GetNavdata(self,data):
        self.Rotations = [data.rotX, data.rotY, data.rotZ]
        self.status = data.state
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",self.Rotations)
        # return self.Rotations


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.seeimage)

    def seeimage(self,data):
        try:
            # calling inside cvbridge
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8") #rgb?
        except CvBridgeError as e:
            print(e)

        # store as a tuple? or the output tuple store individual outputs from the bridge?
        #cv_image.shape returns width, height, and number of colours/channels
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60:   # check is valid image?
            # image,centre,radius,colour
            cv2.circle(cv_image,(50,50),10,255)

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
   # i = 1
   # rospy.spin()
        # i += 1
        # if i % 1000 == 0:
        # print me.Rotations

