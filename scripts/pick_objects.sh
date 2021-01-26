#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch rse_nd_home_service_robot world.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch rse_nd_home_service_robot amcl.launch" &
sleep 3
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch rse_nd_home_service_robot view_navigation.launch" & 
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun pick_objects pick_objects" & 
# xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch"
# xterm  -e  " rosrun rviz rviz -d /home/robond/catkin_ws/src/rse-nd-home-service-robot/rvizConfig/mapping.rviz" 
# rosrun map_server map_saver -f myMap