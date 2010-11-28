#!/usr/bin/env python
import roslib; roslib.load_manifest('wpdtb')
import rospy
import math
import os
#import readline

# torso was set to 0.273470237921 when playing in the lab
import actionlib
import tf
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
    def __init__(self, node_name, side='r'):
        self.user_settle = True
        self.node_name = node_name
        self.side = side
        side_name = 'right' if (side == 'r') else 'left'
        rospy.loginfo("waiting for services")

        ik_info_name="pr2_"+side_name+"_arm_kinematics/get_ik_solver_info"
        get_ik_name ="pr2_"+side_name+"_arm_kinematics/get_ik"

        rospy.wait_for_service(ik_info_name)
        rospy.wait_for_service(get_ik_name)

        self.ik_info_srv = rospy.ServiceProxy(ik_info_name, GetKinematicSolverInfo)
        self.get_ik_srv = rospy.ServiceProxy(get_ik_name, GetPositionIK)

        self.ik_info = self.ik_info_srv()

        self.joints = self.ik_info.kinematic_solver_info.joint_names
        self.num_joints = len(self.joints)

#        print self.joints

        self.ac = {}
        self.ac['l'] = \
            actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action",
                                         JointTrajectoryAction)
        self.ac['r'] = \
            actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action",
                                         JointTrajectoryAction)

        # Wait for joint clients to connect
        if not self.ac['l'].wait_for_server():
            raise Exception("Timout waiting for l_arm_controller")

        if not self.ac['r'].wait_for_server():
            raise Exception("Timout waiting for r_arm_controller")

# Translation from the URDF file:
#
# base_footprint               [ 0        0       0]
# base_link                    [ 0        0       0.051]
#                       fixed
# torso_lift_link              [-0.05     0       0.739675]
#                                                [0.011 to 0.305] prismatic
# r_shoulder_pan_link          [ 0       -0.188   0]
#                                         [-2.135 to 0.565] revolute
# r_shoulder_lift_link         [ 0.1      0       0]
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

#        rospy.spin()
        # Let's skip everything else for now, until I figure out
        # the reachability problem

        self.chesstable = rospy.get_param("chesstable")
        self.chessboard = rospy.get_param("chessboard")
        self.chesspiece = rospy.get_param("chesspiece")
#    print chesstable, chessboard, chesspiece

        self.center_x = self.chessboard['center']['x']
        self.center_y = self.chessboard['center']['y']
        self.center_z = self.chesstable['z'] + self.chessboard['thickness']

        # Set up for IK requests
        self.req = GetPositionIKRequest()
        self.req.timeout = rospy.Duration(5.0)
        self.req.ik_request.ik_link_name = "r_wrist_roll_link"
        self.req.ik_request.ik_seed_state.joint_state.name = self.joints
        self.req.ik_request.ik_seed_state.joint_state.position = \
            [0, 0, -math.pi, 0, 0, -math.pi/2, 0]

        self.req.ik_request.pose_stamped.pose.orientation.w = math.sqrt(2.0)/2.0
        self.req.ik_request.pose_stamped.pose.orientation.x = 0
        self.req.ik_request.pose_stamped.pose.orientation.y = math.sqrt(2.0)/2.0
        self.req.ik_request.pose_stamped.pose.orientation.z = 0

        # Set up for opening and closoing the gripper
        self.gripper_pub = rospy.Publisher("r_gripper_controller/command",
                                           Pr2GripperCommand,
                                           latch=True)

        self.open_gripper_cmd = Pr2GripperCommand()
        self.open_gripper_cmd.position = 2*2*self.chesspiece['radius']
        self.open_gripper_cmd.max_effort = 50

        self.close_gripper_cmd = Pr2GripperCommand()
        self.close_gripper_cmd.position = 0.5*2*self.chesspiece['radius']
        self.close_gripper_cmd.max_effort = 50

        #start tf listener
        self.tf_listener = tf.TransformListener()

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
        joints = [side+"_"+name+"_joint" for name in joint_names]
        reflect = {'l':1, 'r':-1}[side]
        positions   = [[reflect*math.pi/2, math.pi/4, reflect*math.pi/2, -math.pi, 0, 0, 0]]
        return self.move_arm(side, joints, positions, 2.5)

    def move_arm(self, side, joints, positions, duration):
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = []

        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                                velocities = [],
                                                                accelerations = [],
                                                                time_from_start = rospy.Duration((count+1) * duration)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        return self.ac[side].send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

    def compute_target(self, x, y, z, frame_id = "base_footprint"):
        self.req.ik_request.pose_stamped.header.frame_id = frame_id
        self.req.ik_request.pose_stamped.pose.position.x = x
        self.req.ik_request.pose_stamped.pose.position.y = y
        self.req.ik_request.pose_stamped.pose.position.z = z

#        print "compute_target: [%5.3f, %5.3f, %5.3f]" % (x, y, z)

        try:
            resp = self.get_ik_srv(self.req)
        except rospy.ServiceException, e:
            print "get_ik_srv did not process request: %s" % str(e)
            raise

        if resp.error_code.val != resp.error_code.SUCCESS:
            raise Exception("No IK Solution at XYZ=[%f, %f, %f] -- error code %s" % (x, y, z, str(resp.error_code)))
        return [resp.solution.joint_state.name,
                resp.solution.joint_state.position]

    def compute_targets(self):
        # 0.18 is offset from r_gripper_palm_link (== r_wrist_roll_link) to
        # the r_gripper_tool_frame.  For the moment, we're assuming that
        # the r_gripper_tool_frame at the center of the finger tips.  I
        # measured the finger tips to be about 4cm long (3.8cm).  So we
        # add 2cm to the offset to get to the tip of the finger tips.
        gripper_z = 0.18 + 0.02
        pitch = self.chessboard['pitch']

        z_hover = gripper_z              + \
            self.chesstable['z']         + \
            self.chesstable['height']    + \
            self.chessboard['thickness'] + \
            self.chesspiece['length']    + \
            0.02 # 2 cm above top of piece
#        print "z_hover =", z_hover, " ->", z_hover - gripper_z
        self.hover_locs = {}

        z_grasp = gripper_z              + \
            self.chesstable['height']    + \
            self.chesstable['z']         + \
            self.chessboard['thickness'] + \
            self.chesspiece['length']    - \
            0.01 # grasp 1cm below the tip of the piece
#        print "z_grasp =", z_grasp, " ->", z_grasp - gripper_z
        self.grasp_locs = {}

        z_move = gripper_z               + \
            self.chesstable['height']    + \
            self.chesstable['z']         + \
            self.chessboard['thickness'] + \
            self.chesspiece['length']*1.5+ \
            .02 # move 2 cm above top of pieces
#        print "z_move =", z_move, " ->", z_move - gripper_z
        self.move_locs = {}


        for i in range(8):
            for j in range(8):
                pos = 'abcdefgh'[j]+str(i+1)
                x = self.center_x + (i-3.5)*pitch
                y = self.center_y - (j-3.5)*pitch

                self.hover_locs[pos] = self.compute_target(x, y, z_hover)
                self.grasp_locs[pos] = self.compute_target(x, y, z_grasp)
                self.move_locs[pos]  = self.compute_target(x, y, z_move)

    def hover_over(self, pos):
        arm_joints,arm_up_poses = self.move_locs[pos]
        arm_up_poses = list(arm_up_poses) # convert from tuple to list
        shoulder_lift_idx = arm_joints.index(self.side + "_shoulder_lift_joint")
        arm_up_poses[shoulder_lift_idx] = -math.pi/2
        self.move_arm(self.side, arm_joints, [arm_up_poses], 2.5)
        print "Hovering high over", pos
        self.settle()
        loc = self.move_locs[pos]
        print "Moving to hover low over", pos
        self.move_arm(self.side, loc[0], [loc[1]], 1.0)
        print "settling hover low over", pos
        # Allow time for the arm to settle
        self.settle()

    def open_gripper(self):
        self.gripper_pub.publish(self.open_gripper_cmd)
        print "Gripper opening"
#	raw_input("Gripper should be open now.  Press Enter")
#        self.settle()

    def grasp(self, pos):
        loc = self.grasp_locs[pos]
        self.move_arm(self.side, loc[0], [loc[1]], 1.0)
        print "Settling to grasp at", pos
        self.settle()

        self.gripper_pub.publish(self.close_gripper_cmd)
        print "Gripper closing"
#	raw_input("grasp: Gripper should be closed now.  Press Enter")
        self.settle()
        loc = self.move_locs[pos]
        print "Lifting to move_loc", pos
        self.move_arm(self.side, loc[0], [loc[1]], 1.0)
        print "Setting over move loc", pos
        self.settle()

    def place(self, pos):
#        loc = self.move_locs[pos]
#        self.move_arm(self.side, loc[0], [loc[1]], 2.5)
#        print "Settling over move loc", loc
#        self.settle()

        loc = self.grasp_locs[pos]
        self.move_arm(self.side, loc[0], [loc[1]], 1.0)
        print "Settling at grasp_loc", pos
        self.settle()

#        raw_input("Ready to open gripper... please press enter...")
        self.gripper_pub.publish(self.open_gripper_cmd)
#	raw_input("place: Gripper should be open now.  Press Enter...")
        print "Gripper opening"
        self.settle()

    def arm_at_rest(self, pos):
        arm_joints,arm_up_poses = self.move_locs[pos]
        self.move_arm(self.side, arm_joints, [arm_up_poses], 1.0)
        arm_up_poses = list(arm_up_poses) # convert from tuple to list
        shoulder_pan_idx = arm_joints.index(self.side + "_shoulder_pan_joint")
        if self.side == 'r':
            arm_up_poses[shoulder_pan_idx] = -math.pi/2
        else:
            arm_up_poses[shoulder_pan_idx] =  math.pi/2
        self.move_arm(self.side, arm_joints, [arm_up_poses], 2.5)

    def settle(self, dur=1, rate=10):
        r = rospy.Rate(rate)
        if self.user_settle:
            raw_input("Hit enter when the arm has settled...")
        else:
            for i in range(int(dur*rate)):
                if rospy.is_shutdown():
                    break
                try:
                    (trans, rot) = \
                        self.tf_listener.lookupTransform("/base_link",
                                                         self.side + "_gripper_l_finger_tip_frame",
                                                         rospy.Time(0))
                except (ft.LookupException. tf.ConnectivityException):
                    continue
#                print trans, rot
                print "[" + ','.join(["%6.3f" % x for x in trans]) + "]" + \
                    "  [" + ','.join(["%6.3f" % x for x in rot])   + "]"
                r.sleep()
        
def main():
    rospy.init_node("test5")
    frame_id = "base_footprint"
    print "Computing targets..."
    try:
        mover = PieceMoverActionServer("test5")
        mover.compute_targets()
    except Exception, e:
#        print "Exception raised when playing with the mover: %s" % str(e)
        raise

    print "Targets computed, pointing head..."
    mover.aim_head()
    print "..done.  Moving arms out of the way..."
    mover.move_arm_out_of_the_way()
    mover.move_arm_out_of_the_way('r')
    print "...done.  Opening gripper..."
    mover.open_gripper()
    print "...done."

    while not rospy.is_shutdown():
        while True:
            from_pos = raw_input("Select piece location to move from > ")
            if from_pos in mover.grasp_locs:
                break
        while True:
            to_pos   = raw_input("Select piece location to move to   > ")
            if to_pos in mover.grasp_locs:
                break

        print "Moving to hove over location", from_pos, "..."
        mover.hover_over(from_pos)
        print "...done.  Grasping piece..."
        mover.grasp(from_pos)
        print "...done.  Moving to hover over location", to_pos, "..."
        mover.hover_over(to_pos)
        print "...done.  Placing piece..."
        mover.place(to_pos)
        print "...done.  Moving arm out of the way..."
        mover.arm_at_rest(to_pos);
        print "...done."


if __name__ == '__main__':
    rospy.init_node('test5')
    sleep_dur = int(os.getenv("SLEEP_DUR", 3))
    rospy.sleep(sleep_dur) # wait for the world to start
    main()

    # for the moment...
    rospy.spin()
