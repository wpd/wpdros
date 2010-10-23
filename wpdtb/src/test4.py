#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import actionlib
import math

# Brings in the SimpleActionClient
from pr2_common_action_msgs.msg import *

joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]
def main():
    rospy.init_node("test4")
    rospy.loginfo("waiting for services")
    client = actionlib.SimpleActionClient('r_arm_ik', ArmMoveIKAction)
    client.wait_for_server()

    goal = ArmMoveIKGoal()
    goal.tool_frame.header.frame_id = "r_wrist_roll_link"
    goal.pose.header.stamp = rospy.Time.now()
#    goal.pose.header.frame_id = "base_footprint"#"torso_lift_link"
#    goal.pose.pose.position.x = 0.725 # 0.805
#    goal.pose.pose.position.y = -0.175
#    goal.pose.pose.position.z = 1   # 0.55+0.1+fingertip
##    goal.pose.pose.position.x = 0.7
##    goal.pose.pose.position.y = 0
##    goal.pose.pose.position.z = 1
#    goal.pose.pose.orientation.x = 0
#    goal.pose.pose.orientation.y = 0.707
#    goal.pose.pose.orientation.z = 0
#    goal.pose.pose.orientation.w = 0.707

    # goal.pose.header.frame_id = "base_footprint";
    # goal.pose.pose.orientation.x = 0
    # goal.pose.pose.orientation.y = 0.707
    # goal.pose.pose.orientation.z = 0
    # goal.pose.pose.orientation.w = 0.707
    # goal.pose.pose.position.x = 0.767
    # goal.pose.pose.position.y = -0.188
    # goal.pose.pose.position.z = 0.754


    goal.pose.header.frame_id = "torso_lift_link";
    goal.pose.pose.orientation.x = 0
    goal.pose.pose.orientation.y = 0.707
    goal.pose.pose.orientation.z = 0
    goal.pose.pose.orientation.w = 0.707
    goal.pose.pose.position.x = 0.817
    goal.pose.pose.position.y = -0.188
    goal.pose.pose.position.z = -0.048

    goal.ik_timeout = rospy.Duration(5.0)
    goal.ik_seed.name = ["r_" + name for name in joint_names]
    goal.ik_seed.position = [0, 0, -math.pi, 0, 0, -(math.pi/2-.1), 0]
    # (seed positions taken from
    # pr2_arm_move_ik/src/test_clients/pr2_arm_ik_test.cpp)
    goal.move_duration = rospy.Duration(1.0)

    rospy.loginfo("Sending goal");
    client.send_goal(goal)
    rospy.loginfo("result = " + str(client.wait_for_result()))

if __name__ == '__main__':
  main()

