/*Adapted from http://www.ros.org/wiki/navigation/Tutorials/SendingSimpleGoals*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <pr2_common_action_msgs/TuckArmsAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> TuckArmsClient;

void tf_echo()
{
  tf::TransformListener listener;
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  //wait for the listener to get the first message
  listener.waitForTransform("odom_combined", "base_footprint",
			    ros::Time(0), ros::Duration(1000.0));
  while (node.ok()){
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("odom_combined", "base_footprint",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    ROS_INFO("base at (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f)",
	     transform.getOrigin().x(),
	     transform.getOrigin().y(),
	     transform.getOrigin().z(),
#if 0
	     transform.getRotation().getAxis().x(),
	     transform.getRotation().getAxis().y(),
	     transform.getRotation().getAxis().z(),
	     transform.getRotation().getAngle()
#else
	     transform.getRotation().x(),
	     transform.getRotation().y(),
	     transform.getRotation().z(),
	     transform.getRotation().w()
#endif
	     );
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mover2");

  //tell the action clients that we want to spin a thread by default
  TuckArmsClient ta("tuck_arms", true);
  MoveBaseClient ac("pr2_move_base_local", true);

  //wait for the action servers to come up
  while (!ta.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the tuck_arms action server to come up");
  }

  while (!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  pr2_common_action_msgs::TuckArmsGoal tuck;
  tuck.tuck_left  = true;
  tuck.tuck_right = true;
  ROS_INFO("Tucking arms");
  ta.sendGoal(tuck);
  ta.waitForResult();
  if(ta.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the arms are tucked");
  else
    ROS_INFO("The are not tucked for some reason");

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  // Desktop is at (5, 0, 0), size is (0.75, 1.5, 0.10)
  // clearing_radius = 0.59
  // move to (4.1, 0, 0)
  goal.target_pose.pose.position.x = 2.1; // clearing_radius = 0.59
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  boost::thread tf_thread(&tf_echo);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to its target location");
  else
    ROS_INFO("The base failed to move to its target location for some reason");

// shutdown the node and join the thread back before exiting
  ros::shutdown();
  tf_thread.join();

  return 0;
}
