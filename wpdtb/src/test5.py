#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import math

from kinematics_msgs.srv import *

def main():
    rospy.init_node("test5")
    rospy.loginfo("waiting for everything to start")
    rospy.sleep(5)
    rospy.loginfo("waiting for services")

    ik_info_name="pr2_right_arm_kinematics/get_ik_solver_info"
    get_ik_name="pr2_right_arm_kinematics/get_ik"

    rospy.wait_for_service(ik_info_name)
    rospy.wait_for_service(get_ik_name)

    rospy.wait_for_service(ik_info_name)
    rospy.wait_for_service(get_ik_name)

    ik_info_srv = rospy.ServiceProxy(ik_info_name, GetKinematicSolverInfo)
    get_ik_srv = rospy.ServiceProxy(get_ik_name, GetPositionIK)

    ik_info = ik_info_srv()
    num_joints = len(ik_info.kinematic_solver_info.joint_names)

#    for i in range(num_joints):
#        rospy.loginfo("Joint: %d %s" % (i, ik_info.kinematic_solver_info.joint_names[i]))

    req = GetPositionIKRequest()
    req.timeout = rospy.Duration(5.0)
    req.ik_request.ik_link_name = "r_wrist_roll_link"
    req.ik_request.pose_stamped.header.frame_id = "base_footprint"#"torso_lift_link"
    req.ik_request.pose_stamped.pose.position.x = 0.767 # 0.805
    req.ik_request.pose_stamped.pose.position.y = -0.188 #-0.175
    req.ik_request.pose_stamped.pose.position.z = 0.754 # 1   # 0.55+0.1+fingert
    req.ik_request.pose_stamped.pose.orientation.x = 0
    req.ik_request.pose_stamped.pose.orientation.y = 0.707
    req.ik_request.pose_stamped.pose.orientation.z = 0
    req.ik_request.pose_stamped.pose.orientation.w = 0.707
    req.ik_request.ik_seed_state.joint_state.name = \
        ik_info.kinematic_solver_info.joint_names

    req.ik_request.ik_seed_state.joint_state.position = \
        [0, 0, -math.pi, 0, 0, -(math.pi/2-.1), 0]
    try:
        resp = get_ik_srv(req)
    except rospy.ServiceException, e:
        print "get_ik_srv did not process request: %s" % str(e)
        raise

    print resp.error_code

    print resp.error_code.val, resp.error_code.SUCCESS
    if (resp.error_code.val == resp.error_code.SUCCESS):
        for i in range(len(resp.solution.joint_state.name)):
            rospy.loginfo("Joint: %s %f" % (resp.solution.joint_state.name[i], resp.solution.joint_state.position[i]))

if __name__ == '__main__':
  main()

