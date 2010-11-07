#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import math

import actionlib
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

def main():
    rospy.init_node('test')
    
    # Point the head at the finger tip frame wherever it is now.
    aim_head('/r_gripper_l_finger_tip_frame')

    # Now, move the hand to be in front of the center of the robot.
    # The upper arm length is 0.4m.  It is offset from the center of
    # the robot by 0.1m.  The forearm length is 0.321m, but it is flexed
    # up by 0.15 radians (due to the soft limit on the elbow flex
    # joint), so it's length in the X direction is reduced by
    # cos(0.15).  The shoulder pan joint is offset from center (in the
    # Y direction) by 0.188m.  We want to rotate the shoulder pan joint
    # so that the hand is at the center of the robot.
    #
    # We also want to flex the wrist so that the hand is pointed up
    # vertically.  It is already pointed up by 0.15 radians because
    # of the flexed elbow.  So we need to point it up by pi/2 - 0.15.

    dl = 0.5+0.321*math.cos(0.15)
    theta = math.asin(0.188/dl)
    move_arm(positions = [theta,		# shoulder_pan
                          0,			# shoulder_lift
                          0,			# arm_roll
                          -0.15,		# elbow_flex
                          0,			# forearm_roll
                          -(math.pi/2-0.15),	# wrist_flex
                          -theta])		# wrist_roll

    # Point the head at the new position
    aim_head('/r_gripper_l_finger_tip_frame')

# Joint names
joint_names = ["r_shoulder_pan_joint",
               "r_shoulder_lift_joint",
               "r_upper_arm_roll_joint",
               "r_elbow_flex_joint",
               "r_forearm_roll_joint",
               "r_wrist_flex_joint",
               "r_wrist_roll_joint"]

def aim_head(frame_id, x = 0, y = 0, z = 0):
    ac = actionlib.SimpleActionClient('/head_traj_controller/point_head_action',
                                      PointHeadAction)
    ac.wait_for_server()
    g = PointHeadGoal()
    g.target.header.frame_id = frame_id
    g.target.point.x = x
    g.target.point.y = y
    g.target.point.z = z
    g.min_duration = rospy.Duration(.5)
    ac.send_goal(g)
    ac.wait_for_result()

def move_arm(side = 'r', positions = [0, 0, 0, 0, 0, 0, 0]):
    ac = actionlib.SimpleActionClient(side + "_arm_controller/joint_trajectory_action",
                                      JointTrajectoryAction)
    ac.wait_for_server()
    g = JointTrajectoryGoal()
    g.trajectory.joint_names = joint_names
    g.trajectory.points = \
        [JointTrajectoryPoint(positions = positions,
                              velocities = [],
                              accelerations = [],
                              time_from_start = rospy.Duration(2.5))]

    ac.send_goal_and_wait(g, rospy.Duration(30), rospy.Duration(5))

if __name__ == '__main__':
    main()
