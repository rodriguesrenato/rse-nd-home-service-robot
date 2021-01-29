#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/JobRequest.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

ros::Publisher marker_pub;

bool handle_job_request(add_markers::JobRequest::Request &req,
                        add_markers::JobRequest::Response &res)
{
    // Initialize commom marker varibles
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Define marker pose;
    marker.pose = req.pose;

    // Define marker size
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Define marker color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.8f;
    marker.color.a = 1.0;

    // Set infinite lifetime
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

    // Consider any other action different from 'Pickup' will add the marker

    marker.action = visualization_msgs::Marker::ADD;

    if (req.job == "Pickup")
    {
        // Just for ilustrate the pickup, make marker 50% transparent and place it on the
        // robot rear container. 1 second delay just for show this process before delete it
        // marker.action = visualization_msgs::Marker::ADD;
        marker.color.a = 0.5;
        marker_pub.publish(marker);
        ros::Duration(1).sleep();
        marker.action = visualization_msgs::Marker::DELETE;
    }
    else
    {
        // Delete the markert at position
        if (req.job == "Remove")
        {
            marker.action = visualization_msgs::Marker::DELETE;
        }
    }

    // Handle the marker
    marker_pub.publish(marker);

    // Return a feedback response message
    res.msg_feedback = req.job + " job done!";
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Start job_request service to place markers
    ros::ServiceServer service = n.advertiseService("job_request", handle_job_request);

    ROS_INFO("Ready to handle job requests");
    ros::spin();

    return 0;
}