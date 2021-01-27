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

    // Consider any other action different from 'Pickup' will be a 'Drop Off'
    if (req.job == "Pickup" || req.job == "DropOff")
    {
        marker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
        marker.action = visualization_msgs::Marker::DELETE;
    }

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

    // When it is as "Pickup",
    // 1. the robot arrivedat the right place,
    // 2. someone/something puts the marker on the robot -> marker appears
    // 3. wait 5 seconds to this process finishes
    // 4. the robot is ready to deliver the marker -> marker not visible anymore

    // When it is as "DropOff",
    // 1. the robot arrivedat the right place,
    // 2. the robot retrive the marker to someone/something gets it -> marker appears
    // 3. wait 5 seconds to this process finishes
    // 4. someone/something got the marker on the robot -> marker not visible anymore

    // Show the marker
    marker_pub.publish(marker);

    // Wait 5 seconds to simulate a pickup or a drop off
    ros::Duration(5).sleep();

    // Then remove the marker as pickup or dropoff has finished
    if (req.job == "Pickup" || req.job == "DropOff")
    {
        marker.action = visualization_msgs::Marker::DELETE;
    }
    marker_pub.publish(marker);

    // Return a feedback response message
    res.msg_feedback = req.job+" done!";
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