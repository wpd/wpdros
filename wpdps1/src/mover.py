#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdps1')
import rospy
import math
import sys
import tf
from geometry_msgs.msg import Twist

def mover(target_x, target_y, thresh = 0.1):
    pub = rospy.Publisher('/base_controller/command', Twist)
    rospy.init_node('mover')
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
        print "Moving base to (%d, %d)" % (target_x, target_y)
        mover(target_x, target_y)
    except rospy.ROSInterruptException: pass
