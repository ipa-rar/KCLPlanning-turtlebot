#!/usr/bin/env python

import rospy
import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from actionlib_msgs.msg import *
import ar_dock_motion_server.msg #for action server
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion

pub = None


def ar_dock_client():

    client = actionlib.SimpleActionClient("ar_dock_motion_server", ar_dock_motion_server.msg.DockMotionAction)
    #client.wait_for_server()
    while(not client.wait_for_server(rospy.Duration.from_sec(10.0))):
        rospy.loginfo("Waiting for the docking action server to come up")

    goal = ar_dock_motion_server.msg.DockMotionGoal()

#goal.options = goal.MOVE_POSES
    goal.start = True

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(60))
    result = ar_dock_motion_server.msg.DockMotionResult()
    result = client.get_result()

    #status = client.get_status()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Dock Client: Robot have reached the docking station")
        print ("SUCCEEDED")
    else:
        rospy.loginfo("Dock Client: Robot fail to respond")


if __name__ == '__main__':
    try:
        rospy.init_node('ar_dock_client')
        ar_dock_client()

    except rospy.ROSInterruptException:
        rospy.loginfo("Nav Client: program interrupted before completion")
