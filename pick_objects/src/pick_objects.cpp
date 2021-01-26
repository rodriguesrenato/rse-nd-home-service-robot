#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/JobRequest.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::ServiceClient jobRequestClient;

void travelToGoal(MoveBaseClient *ac, const move_base_msgs::MoveBaseGoal goal, const char *goalName)
{
  ROS_INFO("Robot is moving to %s at x:%1.2f,y:%1.2f,yaw:%1.2f ", goalName, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);

  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully arrived at %s", goalName);
    add_markers::JobRequest srv;
    srv.request.action = "Pickup";
    srv.request.goal = goal;
    if (!jobRequestClient.call(srv))
      ROS_ERROR("Failed to call service job_request");
  }
  else
    ROS_INFO("Failed to arrive at %s", goalName);
}

int main(int argc, char **argv)
{
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  jobRequestClient = n.serviceClient<add_markers::JobRequest>("job_request");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goalPickup;

  // set up the frame parameters
  goalPickup.target_pose.header.frame_id = "map";
  goalPickup.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goalPickup.target_pose.pose.position.x = 0.0;
  goalPickup.target_pose.pose.position.y = -2.0;
  goalPickup.target_pose.pose.orientation.w = 1.5707;

  move_base_msgs::MoveBaseGoal goalDropOff;

  // set up the frame parameters
  goalDropOff.target_pose.header.frame_id = "map";
  goalDropOff.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goalDropOff.target_pose.pose.position.x = 0.0;
  goalDropOff.target_pose.pose.position.y = 2.0;
  goalDropOff.target_pose.pose.orientation.w = -1.5707;

  // Send the goal position and orientation for the robot to reach

  travelToGoal(&ac, goalPickup, "Pickup Zone");
  ros::Duration(5).sleep();
  travelToGoal(&ac, goalDropOff, "Drop Off Zone");

  return 0;
}