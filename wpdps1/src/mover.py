#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdps1')
import rospy
import math
import sys
import tf
from geometry_msgs.msg import Twist

def mover(target_x, target_y):
    pub = rospy.Publisher('/base_controller/command', Twist)
    rospy.init_node('mover')
    mytf = tf.TransformListener()
    source = "/odom_combined"
    target = "/base_link"
    mytf.waitForTransform(source, target, rospy.Time(), rospy.Duration(5))
    while not rospy.is_shutdown():
        now = rospy.Time()
        [translation, rotation] = mytf.lookupTransform(source, target, now)
#        str = "hello world %s" % translation
#        rospy.loginfo(str)
#        pub.publish(String(str))
#        print rospy.Time().to_sec()
#        print translation
#        print rotation
        dx = target_x - translation[0]
        dy = target_y - translation[1]
        distance = math.sqrt(dx*dx + dy*dy)
        print distance
        cmd = Twist()
        if distance > 0.01:
            cmd.linear.x=dx # move faster, the further away we are
            cmd.linear.y=dy
        pub.publish(cmd)
        
        rospy.sleep(0.1)
if __name__ == '__main__':
    target_x = 4
    target_y = 0

    if len(sys.argv) >= 2:
        target_x = int(sys.argv[1])
        target_y = int(sys.argv[2])

    try:
        print "Moving base to (%d, %d)" % (target_x, target_y)
        mover(target_x, target_y)
    except rospy.ROSInterruptException: pass
