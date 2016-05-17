import rospy
import roslib; roslib.load_manifest('ardrone_tutorials')
import std_msgs.msg as stMsg
import ardrone_autonomy.msg as dronemsgs
import opencv2
import cv_bridge
import sys
# from sensor_msgs.msg import Image
from ardrone_autonomy.msg import Navdata
# from drone_status import DroneStatus
from cv_bridge import CvBridge, CvBridgeError
# something to to with assigning resources between GUI and ROS topics
# from threading import Lock

# Some Constants
# CONNECTION_CHECK_PERIOD = 250 #ms
# GUI_UPDATE_PERIOD = 20 #ms
# DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


def initdrone():
    rospy.init_node('dronetest',anonymous=True)
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
        self.pubReset = rospy.Publisher("/ardrone/reset",stMsg.Empty)

    # retrive and store data
    def GetNavdata(self,data):
        self.Rotations = [data.rotX, data.rotY, data.rotZ]
        self.status = data.state
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",self.Rotations)
        # return self.Rotations




if __name__ == '__main__':
    initdrone()
    me = BasicDroneController()  # should automatically call GetNavdata when something in received in subscriber
    i = 1
    rospy.spin()
        # i += 1
        # if i % 1000 == 0:
        # print me.Rotations

