#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import math

import actionlib
from kinematics_msgs.srv import *
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *

joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]

class PieceMoverActionServer:
    # We're not really a server yet, but we'll get there eventually
    def __init__(self, node_name, side='right'):
        self.node_name = node_name

        rospy.loginfo("waiting for services")

        ik_info_name="pr2_"+side+"_arm_kinematics/get_ik_solver_info"
        get_ik_name ="pr2_"+side+"_arm_kinematics/get_ik"

        rospy.wait_for_service(ik_info_name)
        rospy.wait_for_service(get_ik_name)

        self.ik_info_srv = rospy.ServiceProxy(ik_info_name, GetKinematicSolverInfo)
        self.get_ik_srv = rospy.ServiceProxy(get_ik_name, GetPositionIK)

        self.ik_info = self.ik_info_srv()

        self.joints = self.ik_info.kinematic_solver_info.joint_names
        self.num_joints = len(self.joints)

        print self.joints

# Translation from the URDF file:
#
# base_footprint               [ 0        0       0]
# base_link                    [ 0        0       0.051]
#                       fixed
# torso_lift_link              [-0.05     0       0.739675]
#                                                [0.011 to 0.305] prismatic
# r_shoulder_pan_link          [ 0       -0.188   0]
#                                         [-2.135 to 0.565] revolute
# r_shoulder_lift_link         [ 0,1      0       0]
#                                 [-0.354 to 1.296] revolute
# r_upper_arm_roll_link        [ 0        0       0]
#                        [-3.750 to 0.650] revolute
# r_upper_arm_link             [ 0        0       0]
#                       fixed
# r_elbow_flex_link            [ 0.4      0       0]
#                                 [-2.121 to -0.150] revolute
# r_forearm_roll_link          [ 0        0       0]
#                     continuous
# r_forearm_link               [ 0        0       0]
#                       fixed
# r_wrist_flex_link            [ 0.321    0       0]
#                                 [-1.994 to -0.100
# r_wrist_roll_link            [ 0        0       0]
#                     continuous
# r_gripper_palm_link          [ 0.07691 -0.01    0]
#                       fixed
# r_gripper_r_finger_link      [ 0.09137 -0.00495 0]
#                                         [0.000 -to- 0.548] revolute (-Z)
# r_gripper_r_finger_tip_link  [ 0        0       0]
#                                         [0.000  to  0.538] revolute (+Z)
# r_gripper_l_finger_tip_frame
#                                 [-0.010 to 0.088] prismatic
#
# Here is what I think.  Let's assume that the torso is all the way
# down.  In that case, the r_shoulder_pan link should be located at
# [-0.05 -0.188 0.801675].  If we were to point the shoulder pan
# joint straight ahead, and set the shoulder lift joint to 0, then
# the r_elbow_flex_link should be located at
# [ 0.45 -0.188 0.801675]
#
# The forearm link is 0.321 long, but it extends at an angle of
# -0.150 radians from the elbow joint, so its effective length
# in the X direction is 0.321*cos(-0.150)=0.31739551601747.
# It angles up in the Z direction by 0.321*sin(0.150)=0.047969640524025
# So the r_wrist_roll_link should be located at
# [ 0.76739... -0.188 -0.188 0.849644640524025]
#
# We could rotate the upper_arm_roll_link by -pi.  In that case, the
# r_wrist_roll_link would be located at
# [ 0.76739... -0.188 -0.188 0.753705359475975]
#
# Or, we could leave it angled up in the Z direction and rotate and
# flex the wrist joints so that the gripper is pointing straight down.
# In order to do that, we would need to rotate the (continuous)
# r_forearm_roll_joint by pi, leaving the r_wrist_roll_link in the
# same position as before, and rotate the r_wrist_flex_joint by
# -(pi/2+0.15).  According to my calculations, the
# r_gripper_l_finger_tip_frame link is [0.16828 -0.01495 0] beyond
# the r_wrist_roll_link, assuming that the X direction is forward
# from the robot.  If we rotate the wrist so it's pointing down, then
# the r_gripper_l_finger_tip_frame link should be at:
#
# [ 0.76739.. -0.17305 0.681364640524025]
# (test2.py params: [0, 0, 0, math.pi, -0.15, -(math.pi/2+0.15), 0])
#
# OK, now, if I were to rotate the r_upper_arm_roll_link by -pi,
# the forearm would angle downward instead of upward.  If I were to
# simulataneously undo the rotation of the wrist roll joint, and compensate
# the -math.pi/2 elbow flex by -0.15 instead of +0.15, I should
# see a net change in the Z direction only of the finger tip frame.
#
# [ 0.76739.. -0.17305 0.585425359475975]
# (test2.py params: [-math.pi, 0, 0, 0, -0.15, -(math.pi/2-0.15), 0]
#
# That was fun.  Suppose I rotated the r_upper_arm_roll_link by only
# -pi/2 (and compensated the rotation of the wrist roll link by pi/2).
# In that situation, the bent forearm would be offset in the +Y
# direction.  The wrist roll link should still be 0.767m from
# base_footprint, (in the +X direction), and 0.048m in (from the
# -0.188 point) in the +Y direction, and at 0.801675:
#
# [ 0.767 -0.140 0.802]
# (test2.py params: [-math.pi/2,0,0,math.pi/2,-0.15,-(math.pi/2-0.00),0]
#
# The finger tip frame should be 0.168m below that; it won't be at the
# same XY coordinates because of the -0.01495 +Y translation of the
# finger tip frame and the rotation of the gripper arising from the
# various rolls.  We can compensate for the rolls by applying a rotation
# of 0.15 radians to the wrist_roll joint and the r_gripper_l_finger_tip_frame
# to line up at
#
# [ 0.767 -0.125 0.634]
# (test2.py params: [-math.pi/2,0,0,-math.pi/2,-0.15,-(math.pi/2-0.00),0.15]
#
# Finally, I should be able to move the Y position back to -0.188 by
# rotating the shoulder pan joint by -atan(-0.048/0.817) -- the 0.817
# instead of 0.767 is due to the fact that the should pan joint sits
# 0.05m back.

# OK, all of that was a little fun, and perhaps even instructive, but
# I'm not really that much closer to answering the question: Where
# should I put the chess board.
#
# Let's put the center of the board at Y=-0.188, that way, it is right
# in front of the right arm.  If we put it at
# Z=0.801675-0.16828=0.633395, then the gripper would just rest on the
# table when it was pointing straight down and the arm was extended as
# far forward as possible.  That "extended as far forward as possible"
# bit would be much easier if the soft limit for the elbow flex joint
# were zero instead of 0.15.  As it is now, when the shoulder_lift
# joint is set to zero, the wrist_roll joint is displaced by
# 0.321*cos(0.15) (0.31739551601747) in the X direction and
# 0.321*sin(0.15) (0.047969640524025) in the Z direction.  We could
# extend the arm as far as possible by rotating the shoulder lift joint
# up by atan2(0.321*sin(0.15), 0.4+0.321*cos(0.15)), at which point
# the wrist roll link should be located at
#
# [0.05+sqrt((0.4+0.321*cos(0.15))^2+(0.321*sin(0.15))^2) -0.188 0.801675]
# [0.768997505429592 -0.188 0.801675]
# (test2.py params: [-math.pi, 0, -math.atan2(0.321*math.sin(0.15), 0.4+0.321*math.cos(0.15)), 0, -0.15, -0.10, 0]
#
# All of that seems to say that we can reach out 0.769m in front of the
# robot.  Rotating the shoulder pan joint will only lessen this distance as
# will raising or lowering the shoulder lift joint.
#
# If (for some reason) I wanted to stay in that same YZ plane, it seems to
# me that, in order to approach the closest to the robot as possible, I would
# want to rotate the upper_arm_roll joint by -pi/2 and simultaneously
# adjust the shoulder_pan and elbow flex joints to pull in the arm
# in a straight line.  The shoulder pan joint would rotate counter clockwise
# until it was at an angle of acos(0.321/0.4).  If the elbow flex
# joint was at -(pi/2+asin(0.321/0.4), the forarm should be parallel to
# the Y axis and the wrist should be at X=0.05+0.4*cos(alpha),
# Y=-0.188, Z=0.801675
#
# The problem is, I can't rotate the elbow flex joint to -(pi/2+asin(...)).
# It hits its soft limit of -2.121 

        rospy.spin()
        # Let's skip everything else for now, until I figure out
        # the reachability problem

        self.chesstable = rospy.get_param("chesstable")
        self.chessboard = rospy.get_param("chessboard")
        self.chesspiece = rospy.get_param("chesspiece")
#    print chesstable, chessboard, chesspiece

        self.center_x = self.chessboard['center']['x']
        self.center_y = self.chessboard['center']['y']
        self.center_z = self.chesstable['z'] + self.chessboard['thickness']

    def aim_head(self):
        client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        client.wait_for_server()
        g = PointHeadGoal()
        g.target.header.frame_id = '/base_footprint'
   #g.target.header.frame_id = '/odom_combined'
        g.target.point.x = self.center_x
        g.target.point.y = self.center_y
        g.target.point.z = self.center_z
        g.min_duration = rospy.Duration(.1)

        client.send_goal(g)
        client.wait_for_result()

    def move_arm_out_of_the_way(self, side='l'):
        ac = actionlib.SimpleActionClient(side + "_arm_controller/joint_trajectory_action", JointTrajectoryAction)

        # Wait for joint client to connect with timeout
        if not ac.wait_for_server(rospy.Duration(30)):
            rospy.logerr("timeout waiting for " + side + " arm joint trajectory action")
            return False

        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
        goal.trajectory.points = []
        positions = [[({'l':1, 'r':-1}[side]) * math.pi/4, 0, 0, 0, 0, 0, 0]]

        move_duration = 2.5
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                                velocities = [],
                                                                accelerations = [],
                                                                time_from_start = rospy.Duration((count+1) * move_duration)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        return ac.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def compute_targets(self, frame_id):
        req = GetPositionIKRequest()
        req.timeout = rospy.Duration(5.0)
        req.ik_request.ik_link_name = "r_wrist_roll_link"
        req.ik_request.pose_stamped.header.frame_id = frame_id
        # Seed IK with the arm reaching over the table
        req.ik_request.ik_seed_state.joint_state.name = \
            self.joints
        req.ik_request.ik_seed_state.joint_state.position = \
            [0, 0, -math.pi, 0, 0, -math.pi/2, 0]

        self.hover_locs = {}

        z_hover = \
            self.chesstable['z']         + \
            self.chesstable['height']    + \
            self.chessboard['thickness'] + \
            self.chesspiece['length']    + \
            0.01 # 1 cm above top of piece

        z_grasp = \
            self.chesstable['height']    + \
            self.chesstable['z']         + \
            self.chessboard['thickness'] + \
            self.chesspiece['length'] / 2 # middle of piece

        z_move = \
            self.chesstable['height']    + \
            self.chesstable['z']         + \
            self.chessboard['thickness'] + \
            self.chesspiece['length']*1.5+ \
            -.02 # move 2 cm above top of pieces

        pitch = self.chessboard['pitch']
        self.hover = {}
        gripper_z = 0 # 0.07 # guess for the moment
        for i in range(8):
            for j in range(8):
                req.ik_request.pose_stamped.pose.position.x = self.center_x + (i-3.5)*pitch
                req.ik_request.pose_stamped.pose.position.y = self.center_y + (j-3.5)*pitch
                req.ik_request.pose_stamped.pose.position.z = z_hover+gripper_z

                print req.ik_request.pose_stamped.pose.position
                
                try:
                    resp = self.get_ik_srv(req)
                except rospy.ServiceException, e:
                    print "get_ik_srv did not process request: %s" % str(e)
                    raise

                if resp.error_code.val != resp.error_code.SUCCESS:
                    raise Exception("No IK Solulion -- error code %s" % str(resp.error_code))
                pos = 'abcdefgh'[i]+str(j+1)
                self.hover[pos] = [resp.solution.joint_state.name,
                                   resp.solution.joint_state.position]

        print self.hover

def main():
    rospy.init_node("test5")
    frame_id = "base_footprint"
    try:
        mover = PieceMoverActionServer("test5")
#        print "Pointing head..."
#        mover.aim_head()
#        print "done.  Moving arm out of the way"
#        mover.move_arm_out_of_the_way()
#        print "done."

        # Compute the target locations relative to the base footprint
        mover.compute_targets(frame_id)
    except Exception, e:
#        print "Exception raised when playing with the mover: %s" % str(e)
        raise
if __name__ == '__main__':
    rospy.sleep(30) # wait for the world to start

    main()

    # for the moment...
    rospy.spin()
