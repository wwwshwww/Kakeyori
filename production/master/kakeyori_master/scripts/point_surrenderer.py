#!/usr/bin/env python
from kakeyori_master.srv import *
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
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.update)

    def update(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.theta = data.pose.pose.orientation.z

def normalizeTheta(theta):
    t = 0
    if theta < 0:
        t = 2 * math.pi + theta
    elif theta > 2 * math.pi:
        t = theta % (2 * math.pi)
    else:
        t = theta
    return t

class setGoal():
    def __init__(self):
        self.world = coordinatesSystem()
        self.sr = rospy.Service('point_surrenderer', RelativeCoordinates, self.handleRelative)

    def getGoalCoordinates(self, rela_x, rela_r, rela_theta):
        # to be normalized
        angle = normalizeTheta(rela_theta + self.world.theta)
        x = self.world.x + rela_r * math.cos(angle) / 100
        y = self.world.y + rela_r * math.sin(angle) / 100
        return x, y, angle

    def movebaseClient(self, x, y, theta):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = theta

        client.send_goal(goal)
        wait = client.wait_for_result()
        
        if not wait:
            rospy.logerr("server not available!")
            rospy.signal_shutdown("server not available!")
        else:
            return client.get_result()

    def handleRelative(self, req):
        goal_x, goal_y, goal_angle = self.getGoalCoordinates(req.x, req.y, req.theta)
        return self.movebaseClient(goal_x, goal_y, goal_angle)

def main():
    rospy.init_node('point_surrenderer', anonymous=True)
    sg = setGoal()
    print('ready')
    rospy.spin()

if __name__ == "__main__":
    main()