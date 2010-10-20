#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdchess')
import rospy;

# Brings in the SimpleActionClient
import actionlib
#from pr2_common_action_msgs.msg import TuckArmsAction,TuckArmsGoal
from pr2_common_action_msgs.msg import *

def tuckarms():
    ta = actionlib.SimpleActionClient('tuck_arms', TuckArmsAction)
    tuck = TuckArmsGoal()
    tuck.tuck_left  = True
    tuck.tuck_right = True
#    rospy.loginfo("Tucking Arms: Waiting for server")
    ta.wait_for_server()
#    rospy.loginfo("Tucking Arms: Sending goal")
    ta.send_goal(tuck)
#    rospy.loginfo("Tucking Arms: Waiting for result")
    ta.wait_for_result()
    result = ta.get_result()
#    rospy.loginfo("Tucking Arms: Result: {%s, %s}" % (result.tuck_left, result.tuck_right))

def main():
    rospy.init_node("test1")
    rospy.loginfo("Ticking Arms");
    tuckarms()
    rospy.loginfo("Arms tucked");

if __name__ == '__main__':
  main()

