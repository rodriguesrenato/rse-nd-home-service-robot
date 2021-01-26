#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch  turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" & 
sleep 3
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch gmapping slam_gmapping_pr2.launch " & 
sleep 3
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
# xterm  -e  " rosrun rviz rviz -d /home/robond/catkin_ws/src/rse-nd-home-service-robot/rvizConfig/mapping.rviz" 
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py