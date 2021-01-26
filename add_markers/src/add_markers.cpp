#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/JobRequest.h"
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <tf/tf.h>

ros::Publisher marker_pub;

bool handle_job_request(add_markers::JobRequest::Request &req,
                        add_markers::JobRequest::Response &res)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Consider any other action different from 'Pickup' will be a 'Drop Off'
    if (req.action == "Pickup")
    {
        marker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
        marker.action = visualization_msgs::Marker::DELETE;
    }

    // geometry_msgs::Pose robotPose = req.goal.target_pose.pose;
    // tf::Quaternion q(
    // robotPose.orientation.x,
    // robotPose.orientation.y,
    // robotPose.orientation.z,
    // robotPose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // ROS_INFO()
    // tf::Quaternion q = robotPose.orientation; 
    // double yaw = tf::getYaw(q);

    marker.pose = req.goal.target_pose.pose;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    ros::Duration(5).sleep();
    marker_pub.publish(marker);
    ros::Duration(5).sleep();
    if (req.action == "Pickup")
    {
        marker.action = visualization_msgs::Marker::DELETE;
    }
    marker_pub.publish(marker);

    // Return a feedback response message
    res.msg_feedback = "OK";
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    // ros::Rate r(1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::ServiceServer service = n.advertiseService("job_request", handle_job_request);
    ROS_INFO("Ready to handle job requests");
    ros::spin();

    return 0;
}