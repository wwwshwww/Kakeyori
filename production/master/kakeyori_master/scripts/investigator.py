#!/usr/bin/env python
import sys
from kakeyori_master.srv import *
import rospy

def client(x, y, theta):
    rospy.wait_for_service('point_surrenderer')
    try:
        point_surrenderer = rospy.ServiceProxy('point_surrenderer', RelativeCoordinates)
        resp = point_surrenderer(x, y, thata)
        return resp.isGoal
    except rospy.ServiceException, e:
        print("Service call faild: %s"%e)