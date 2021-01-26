#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=`rospack find rse_nd_home_service_robot`/map/restaurant_renato.world 3d_sensor:=r200" &
# sleep 5
# xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py" & 
sleep 3
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=`rospack find rse_nd_home_service_robot`/map/restaurant_renato.yaml 3d_sensor:=r200" & 
# xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch rse_nd_home_service_robot amcl.launch" &
sleep 3
xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch rse_nd_home_service_robot view_navigation.launch" & 
# xterm  -e  " source /opt/ros/kinetic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch"
# xterm  -e  " rosrun rviz rviz -d /home/robond/catkin_ws/src/rse-nd-home-service-robot/rvizConfig/mapping.rviz" 
# rosrun map_server map_saver -f myMap