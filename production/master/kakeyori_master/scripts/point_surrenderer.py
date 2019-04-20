#!/usr/bin/env python
from kakeyori_master.srv import *
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math
import tf

class coordinatesSystem():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.update)

    def update(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        e = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
        self.theta = e[2]
        #print(math.degrees(self.theta))

        # thanks https://www.kazetest.com/vcmemo/quaternion/quaternion.htm
        # print('now:', self.x, self.y, math.degrees(self.theta))

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

    def getGoalCoordinates(self, rela_x, rela_y, rela_theta):
        # to be normalized
        angle = rela_theta + self.world.theta
        r = math.sqrt(rela_x ** 2 + rela_y ** 2)
        # y = self.world.y + r * math.cos(angle) / 100
        # x = self.world.x + r * math.sin(angle) / 100
        x = self.world.x + 0.5
        y = self.world.y
        print('set goal:', x, y, 'deg:', math.degrees(angle))
        print('world:', self.world.x, self.world.y)
        return x, y, angle

    def movebaseClient(self, x, y, theta):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        q = tf.transformations.quaternion_from_euler(0, 0, theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        client.send_goal(goal)
        wait = client.wait_for_result()

        if not wait:
            rospy.logerr("server not available!")
            rospy.signal_shutdown("server not available!")
        else:
            return client.get_result()

    def handleRelative(self, req):
        goal_x, goal_y, goal_angle = self.getGoalCoordinates(req.x, req.y, req.theta)
        result = self.movebaseClient(goal_x, goal_y, goal_angle)
        if result:
            rospy.loginfo('Goal!')
            return True
        else:
            rospy.loginfo('Faild.')
            return False

def main():
    rospy.init_node('point_surrenderer_', anonymous=True)
    sg = setGoal()
    print('Ready')
    rospy.spin()

if __name__ == "__main__":
    main()