#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/JobRequest.h"
#include <tf/tf.h>
#include <math.h>
#include <nav_msgs/Odometry.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
float xRobotContainerOffset = 0.25;

// Service for request to add markers
ros::ServiceClient jobRequestClient;


geometry_msgs::Pose getRobotContainerPose()
{
  geometry_msgs::Pose containerPose;
  boost::shared_ptr<nav_msgs::Odometry const> sharedOdom;
  nav_msgs::Odometry odom;
  sharedOdom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(5));
  if (sharedOdom != NULL)
  {
    odom = *sharedOdom;
    ROS_INFO("Got robot odometry, calculate the right robot container center");

    tf::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    containerPose.position.x = odom.pose.pose.position.x - xRobotContainerOffset * sin(yaw);
    containerPose.position.y = odom.pose.pose.position.y - xRobotContainerOffset * cos(yaw);
    containerPose.position.z = odom.pose.pose.position.z;
    containerPose.orientation.x = odom.pose.pose.orientation.x;
    containerPose.orientation.y = odom.pose.pose.orientation.y;
    containerPose.orientation.z = odom.pose.pose.orientation.z;
    containerPose.orientation.w = odom.pose.pose.orientation.w;

    ROS_INFO("roll:%1.2f, pitch:%1.2f, yaw:%1.2f", roll, pitch, yaw);
    ROS_INFO("x:%1.2f, y:%1.2f, z:%1.2f", containerPose.position.x, containerPose.position.y, containerPose.position.z);
  }
  return containerPose;
}


void travelToGoal(MoveBaseClient *ac, const move_base_msgs::MoveBaseGoal goal, const char *goalName)
{
  ROS_INFO("Robot is moving to %s at x:%1.2f,y:%1.2f,yaw:%1.2f ", goalName, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);

  // point the robot goal pose
  ac->sendGoal(goal);

  // Wait an infinite time for the results
  ac->waitForResult();

  // Check if the robot reached its goal
  if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully arrived at %s zone", goalName);

    // Request to add_marker to handle the marker
    add_markers::JobRequest srv;
    srv.request.job = goalName;
    srv.request.pose = getRobotContainerPose();//goal.target_pose.pose;
    if (!jobRequestClient.call(srv))
      ROS_ERROR("Failed to call service job_request");
  }
  else
    ROS_INFO("Failed to arrive at %s zone", goalName);
}


move_base_msgs::MoveBaseGoal setupGoal(float x, float y, float z, float roll, float pitch, float yaw)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  goal.target_pose.pose.orientation.x = q[0];
  goal.target_pose.pose.orientation.y = q[1];
  goal.target_pose.pose.orientation.z = q[2];
  goal.target_pose.pose.orientation.w = q[3];
  return goal;
}

int main(int argc, char **argv)
{
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  getRobotContainerPose();
  // Connect to the JobRequest service
  jobRequestClient = n.serviceClient<add_markers::JobRequest>("job_request");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to start
  while (!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to start");
  }

  // Define goalPickup
  move_base_msgs::MoveBaseGoal goalPickup;
  goalPickup = setupGoal(1.0, 1.0, 0.0, 0.0, 0.0, 1.15707);
  // goalPickup.target_pose.header.frame_id = "map";
  // goalPickup.target_pose.header.stamp = ros::Time::now();
  // goalPickup.target_pose.pose.position.x = 0.0;
  // goalPickup.target_pose.pose.position.y = -2.0;
  // tf::Quaternion q;
  // q.setRPY(0.0, 0.0, -1.5707);
  // q.normalize();
  // goalPickup.target_pose.pose.orientation.x = q[0];
  // goalPickup.target_pose.pose.orientation.y = q[1];
  // goalPickup.target_pose.pose.orientation.z = q[2];
  // goalPickup.target_pose.pose.orientation.w = q[3];

  // Define goalDropOff
  move_base_msgs::MoveBaseGoal goalDropOff;
  goalDropOff = setupGoal(-0.5, -0.4, 0.0, 0.0, 0.0, -1.15707);

  // goalDropOff.target_pose.header.frame_id = "map";
  // goalDropOff.target_pose.header.stamp = ros::Time::now();
  // goalDropOff.target_pose.pose.position.x = 0.0;
  // goalDropOff.target_pose.pose.position.y = 2.0;

  // tf::Quaternion qDropOff;
  // qDropOff.setRPY(0.0, 0.0, -1.5707);
  // qDropOff.normalize();
  // goalDropOff.target_pose.pose.orientation.x = qDropOff[0];
  // goalDropOff.target_pose.pose.orientation.y = qDropOff[1];
  // goalDropOff.target_pose.pose.orientation.z = qDropOff[2];
  // goalDropOff.target_pose.pose.orientation.w = qDropOff[3];

  // tf::Matrix3x3 m(q);

  //   tf::Matrix3x3 m2;
  //   m2.setRotation(q);

  //   /**< rotation Matrix - > quaternion */
  //   m.getRotation(q);

  //   /**< rotation Matrix -> rpy */
  //   double roll, pitch, yaw;
  //   m.getRPY(roll, pitch, yaw);

  //   /**< rpy -> quaternion */
  //   tf::Quaternion q3;
  //   q3.setRPY(roll, pitch, yaw);
  //   q3.normalize();

  // tf::Quaternion q(
  //         msg->pose.pose.orientation.x,
  //         msg->pose.pose.orientation.y,
  //         msg->pose.pose.orientation.z,
  //         msg->pose.pose.orientation.w);
  //     tf::Matrix3x3 m(q);
  //     double roll, pitch, yaw;
  //     m.getRPY(roll, pitch, yaw);

  // Start robot duties
  // Go to the goalPickup
  travelToGoal(&ac, goalPickup, "Pickup");

  // wait 2 seconds
  ros::Duration(2).sleep();

  // Go to the goalDropOff
  travelToGoal(&ac, goalDropOff, "DropOff");

  return 0;
}