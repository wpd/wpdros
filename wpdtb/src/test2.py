#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdchess')
import rospy
import math

# Brings in the SimpleActionClient
import actionlib
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

# Joint names
joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]
#joint_names = ["shoulder_pan"]

def movearm(side):
    ac = actionlib.SimpleActionClient(side + "_arm_controller/joint_trajectory_action", JointTrajectoryAction)

    # Wait for joint client to connect with timeout
    if not ac.wait_for_server(rospy.Duration(30)):
        rospy.logerr("timeout waiting for " + side + " arm joint trajectory action")
        return False

    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
    goal.trajectory.points = []
    positions = [[({'l':1, 'r':-1}[side]) * math.pi/4, 0, 0, 0, 0, 0, 0]]
#    positions = [[({'l':1, 'r':-1}[side]) * math.pi/4]]
    move_duration = 2.5
    for p, count in zip(positions, range(0,len(positions)+1)):
        goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                            velocities = [],
                                                            accelerations = [],
                                                            time_from_start = rospy.Duration((count+1) * move_duration)))
    goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
    print type(goal.trajectory.points), len(goal.trajectory.points), type(goal.trajectory.points[0])
#    return 0
    return ac.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
def main():
    rospy.init_node("test2")
    rospy.loginfo("Moving arm")
    rospy.loginfo("result = " + str(movearm('l')))

if __name__ == '__main__':
  main()

