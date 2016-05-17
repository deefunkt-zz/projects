#!/usr/bin/env python

import rospy
import roslib
import std_msgs.msg as stMsg
import ardrone_autonomy.msg as dronemsgs
import ardrone_autonomy.srv as droneservices


def initdrone():
    rospy.init_node('dronetest', anonymous=True)
    rospy.Subscriber("/ardrone/navdata", dronemsgs.Navdata, callback)
    takeoffpub = rospy.Publisher("/ardrone/takeoff", stMsg.Empty)
    # takeoffpub.publish(stMsg.Empty)
    rospy.spin()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':
    try:
        initdrone()
    except rospy.ROSInterruptException:
        pass
