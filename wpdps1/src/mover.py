#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdps1')
import rospy
import math
import sys
import tf

#from trajectory_msgs.msg import *
#from actionlib_msgs.msg import *
#from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
import actionlib
from pr2_tuck_arms_action import tuck_arms_main

from geometry_msgs.msg import Twist

def mover(target_x, target_y, thresh = 0.1):
    nodename='mover'
    pub = rospy.Publisher('/base_controller/command', Twist)
    rospy.init_node(nodename)

    # goal = TuckArmsGoal()
    # goal.tuck_left = True
    # goal.tuck_right = True

    # rospy.logdebug('about to sleep')
    # tuck_arms_action_server = tuck_arms_main.TuckArmsActionServer('tuck_arms')
    # rospy.sleep(0.001)  # wait for time
    # tuck_arm_client = actionlib.SimpleActionClient('tuck_arms',
    #                                                TuckArmsAction)
    # rospy.logdebug('Waiting for action server to start')
    # tuck_arm_client.wait_for_server(rospy.Duration(10.0))
    # rospy.logdebug('Sending goal to action server')
    # tuck_arm_client.send_goal(goal)


    mytf = tf.TransformListener()
    source = "/odom_combined"
    target = "/base_link"
    mytf.waitForTransform(source, target, rospy.Time(), rospy.Duration(5))
    while not rospy.is_shutdown():
        now = rospy.Time()
        [translation, rotation] = mytf.lookupTransform(source, target, now)
        dx = target_x - translation[0]
        dy = target_y - translation[1]
        distance = math.sqrt(dx*dx + dy*dy)
        cmd = Twist()
        if distance > thresh:
            # move faster, the further away we are
            cmd.linear.x=dx
            cmd.linear.y=dy
        else:
            break
        pub.publish(cmd)
        print distance,dx,dy,cmd.linear.x, cmd.linear.y
        
        rospy.sleep(0.1)
if __name__ == '__main__':
    target_x = 4.5
    target_y = 0

    if len(sys.argv) >= 2:
        target_x = int(sys.argv[1])
        target_y = int(sys.argv[2])

    try:
        print "Moving base to (%f, %f)" % (target_x, target_y)
        mover(target_x, target_y)
    except rospy.ROSInterruptException: pass
