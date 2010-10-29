#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import math

# Brings in the SimpleActionClient
import actionlib
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

# Joint names, in order reported by /joint_states topic
joint_names = ["upper_arm_roll",
               "shoulder_pan", 
               "shoulder_lift",
               "forearm_roll",
               "elbow_flex", 
               "wrist_flex", 
               "wrist_roll" ]
#joint_names = ["shoulder_pan"]

def movearm(side = 'r', positions = [[0, 0, 0, 0, 0, 0, 0]]):
    ac = actionlib.SimpleActionClient(side + "_arm_controller/joint_trajectory_action", JointTrajectoryAction)

    # Wait for joint client to connect with timeout
    if not ac.wait_for_server(): # rospy.Duration(30)):
        rospy.logerr("timeout waiting for " + side + " arm joint trajectory action")
        return False

    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
    goal.trajectory.points = []
#    positions = [[0, 0, math.pi, 0, 0, 0, 0]]
#    positions = [[({'l':1, 'r':-1}[side]) * math.pi/4]]
    move_duration = 2.5
    for p, count in zip(positions, range(0,len(positions)+1)):
        goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                            velocities = [],
                                                            accelerations = [],
                                                            time_from_start = rospy.Duration((count+1) * move_duration)))
    goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
#    print type(goal.trajectory.points), len(goal.trajectory.points), type(goal.trajectory.points[0])
#    return 0
    return ac.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
def main():
    rospy.init_node("test2")
    rospy.loginfo("Moving arms")
    rospy.loginfo("result = " + str(movearm('l', [[0, math.pi/2, 0, 0, -0.10, -.15, 0]])))
#    rospy.loginfo("result = " + str(movearm('r', [[0, 0, 0, 0, 0, 0, 0]])))
    rospy.loginfo("result = " + \
                      str(movearm('r', [[-math.pi/2,	# upper_arm_roll
                                          -.1,		# shoulder_pan
                                          0, 		# shoulder_lift
                                          0,		# forarm_roll
                                          -(math.pi/2+.1),	# elbow_flex
                                         -0.10,		# wrist_flex
                                          0]])))	# wrist_roll

                      # str(movearm('r', [[-math.pi/2,	# upper_arm_roll
                      #                     0,		# shoulder_pan
                      #                     0, 		# shoulder_lift
                      #                     0,		# forarm_roll
                      #                    -(math.pi/2),	# elbow_flex
                      #                    -0.10,		# wrist_flex
                      #                     0]])))	# wrist_roll

if __name__ == '__main__':
  main()

