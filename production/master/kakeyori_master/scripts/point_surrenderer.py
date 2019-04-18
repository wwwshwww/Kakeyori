#!/usr/bin/env python
import kakeyori_master.srv import *
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math

class coordinatesSystem():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, update)

    def update(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.theta = data.pose.pose.orientation.z)

def getGoalCoordinates(world, rela_x, rela_r, rela_theta):
    # to be normalized
    angle = rela_theta + world.theta
    x = rela_r * math.cos(angle)
    y = rela_r * math.sin(angle)
    x /= 100 # cm to m
    y /= 100
    return x, y, angle

def movbaseClient(x, y, theta):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = theta
    client.send_goal(goal)

    if not wait:
        rospy.logerr("server not available!")
        rospy.signal_shutdown("server not available!")
    else:
        return client.get_result()

def handleRelative(req):
    goal_x, goal_y, goal_angle = getGoalCoordinates(req.x, req.r, req.theta)
    return movebaseClient(goal_x, goal_y, goal_angle)

def main():
    rospy.init_node('point_surrenderer', anonymous=True)
    wc = coordinatesSystem()
    sr = rospy.Service('point_surrenderer', RelativeCoordinates, handleRelative)
    print('ready')
    rospy.spin()

if __name__ == "__main__":
    main()