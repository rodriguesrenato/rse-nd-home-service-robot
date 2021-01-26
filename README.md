# rse-nd-home-service-robot
Course project "Home Service Robot" repository for the Udacity Robotics Software Engineer Nanodegree program.

This project contains the following Catkin packages :

   
## Installation
Clone this repository in **src** folder in your catkin workspace and following repositories bellow:
```
cd ~/catkin_ws/src
git clone https://github.com/rodriguesrenato/rse-nd-home-service-robot.git
git clone https://github.com/rodriguesrenato/rse-nd-renato-robot.git [TODO]
```
and the following official packages:
```
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_interactions.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
git clone https://github.com/ros-teleop/teleop_twist_keyboard
```

## Usage
Supposing your catkin workspace is located in `~/`, source your workspace:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
SLAM and save your map
1. Go to `rse-nd-home-service-robot/scripts` and run `./test_slam.sh`.
2. Navigate with teleop keyboard through your map until mapping most of the areas.
3. Go to `rse-nd-home-service-robot/map` and save your map by running `rosrun map_server map_saver -f restaurant_renato`. 

Test your SLAM map and navigate through it
1. Go to `rse-nd-home-service-robot/scripts` and run `./test_amcl.sh`.
2. In Rviz, select a point in the map with `2D NAV Goal` cursor to make the robot navigate to it.

## License
The contents of this repository are covered under the MIT License.
