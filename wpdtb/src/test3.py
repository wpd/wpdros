#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import math

# Brings in the SimpleActionClient
import actionlib
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from kinematics_msgs.srv import *
from sensor_msgs.msg import JointState
# Joint names
joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]
#joint_names = ["shoulder_pan"]

def movearm(side = 'r', positions = [[0, 0, 0, 0, 0, 0, 0]]):
    ac = actionlib.SimpleActionClient(side + "_arm_controller/joint_trajectory_action", JointTrajectoryAction)

    # Wait for joint client to connect with timeout
    if not ac.wait_for_server():
        rospy.logerr("timeout waiting for " + side + " arm joint trajectory action")
        return False

    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
    goal.trajectory.points = []
#    positions = [[({'l':1, 'r':-1}[side]) * math.pi/4, 0, 0, 0, 0, 0, 0]]

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
joint_states = []
def update_joint_positions(msg):
    global joint_states
    joint_states = msg.position[16:23]

def main():
    rospy.init_node("test3")
    rospy.loginfo("waiting for services")
#    rospy.Subscriber("/joint_states", JointState, update_joint_positions)
#    rospy.sleep(0.2) # wait for joint positions

    ik_info_name="pr2_right_arm_kinematics/get_ik_solver_info"
    get_ik_name="pr2_right_arm_kinematics/get_ik"

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
#    req.ik_request.pose_stamped.pose.position.x = 0.767 # 0.805
#    req.ik_request.pose_stamped.pose.position.y = -0.188 #-0.175
#    req.ik_request.pose_stamped.pose.position.z = 0.754 # 1   # 0.55+0.1+fingertip
    req.ik_request.pose_stamped.pose.position.x = 0.767
    req.ik_request.pose_stamped.pose.position.y = -0.140
    req.ik_request.pose_stamped.pose.position.z = 0.802
    print req.ik_request.pose_stamped.pose.position

    req.ik_request.pose_stamped.pose.orientation.x = 0
    req.ik_request.pose_stamped.pose.orientation.y = math.sqrt(2.0)/2.0
#    req.ik_request.pose_stamped.pose.orientation.y = 0
    req.ik_request.pose_stamped.pose.orientation.z = 0
    req.ik_request.pose_stamped.pose.orientation.w = math.sqrt(2.0)/2.0
#    req.ik_request.pose_stamped.pose.orientation.w = 1
    req.ik_request.ik_seed_state.joint_state.name = \
        ik_info.kinematic_solver_info.joint_names

#    print ik_info.kinematic_solver_info.limits
#    joint_snap = joint_states
#    print "Could initialize seed states with", joint_snap

#    for i in range(num_joints):
#        req.ik_request.ik_seed_state.joint_state.position.append((ik_info.kinematic_solver_info.limits[i].min_position + 
#                                                                  ik_info.kinematic_solver_info.limits[i].max_position)/2.0)
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
        print movearm(side='r', positions=[resp.solution.joint_state.position])
#        print movearm()

if __name__ == '__main__':
  main()

