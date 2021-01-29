# rse-nd-home-service-robot
Course project "Home Service Robot" repository for the Udacity Robotics Software Engineer Nanodegree program.

This project consist to build a service robot that navigates through of the world `restaurant_renato`, picking up or droping off a marker on the respective goal locations. It constains scripts to build the map using gmapping SLAM, to localize itself with AMCL, navigate to predeterminated goal locations and interact with markers in Rviz.

## Installation
Assuming your catkin workspace `catkin_ws` is located in `~/`, clone this repository and the official repositories bellow in **src** folder of your catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/rodriguesrenato/rse-nd-home-service-robot.git
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/ros-teleop/teleop_twist_keyboard
```
Then build and source it:
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
## Usage

- Generate the map files of your environment using SLAM:
1. Go to `rse-nd-home-service-robot/scripts` and run `./test_slam.sh`.
2. Navigate robot using teleop keyboard through your map until cover most of the areas. If results is not good enough, adjusts gmapping params in `/home/robond/catkin_ws/src/rse-nd-home-service-robot/renato_robot/launch/slam_gmapping.launch`.
3. Go to `rse-nd-home-service-robot/map` and save your map by running in a new terminal: `rosrun map_server map_saver -f restaurant_renato`. 

- Test your SLAM map and navigate through it using `2D NAV Goal`:
1. Go to `rse-nd-home-service-robot/scripts` and run `./test_amcl.sh`.
2. In Rviz, select a point in the map with `2D NAV Goal` cursor to make the robot navigate to it.

- Run the full Home Service Robot
1. Go to `rse-nd-home-service-robot/scripts` and run `./home_service.sh`.

## License
The contents of this repository are covered under the MIT License.
