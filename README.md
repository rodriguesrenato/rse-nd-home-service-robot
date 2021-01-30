# rse-nd-home-service-robot
Course project "Home Service Robot" repository for the Udacity Robotics Software Engineer Nanodegree program.

This project consist of building a service robot that navigates through of the world `restaurant_renato`, picking up or droping off a marker on the respective goal locations. It constains scripts to build the map using gmapping SLAM, to localize itself with AMCL, navigate to predeterminated goal locations and interact with markers in Rviz.

The packages above were implemented:

- `renato_robot`: The main package that contains the robot, world file, maps and launch files for SLAM and localization.
  - URDF robot `renato_robot`.
  - World file `restaurant_renato.world`.
  - Prebuilt map with **gmapping**.
  - Navigation configuration params.
  - Launch files:
    - `world.launch`: Initialize the world and robot on it.
    - `slam_gmapping`: Run **gmapping** with preconfigured params.
    - `amcl.launch`: Run **amcl** and **move_base** with preconfigured params.
    - `view_navigation.launch`: Run Rviz with preconfigured params.
    
- `pick_objects`: Contains a script `pick_objects.cpp` that navigate the robot through predefined goal positions.

- `add_markers`: Contains a script `add_markers.cpp` that start a service which handle marker on the map. The service receive a `job` and `pose` on request, the `job` variable is a string that specify the action with marker at the `pose` position.

External Official package were used: `slam_gmapping` and `teleop_twist_keyboard`.

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
To run script files, make them executable first:
```
cd ~/catkin_ws/src/rse-nd-home-service-robot/scripts
chmod +x *.sh
```

## Usage

- Generate the map files of your environment using SLAM:
  1. Go to `rse-nd-home-service-robot/scripts` and run `./test_slam.sh`. This script will run `world.launch`,`gmapping.launch`.
  2. Navigate robot using teleop keyboard through your map until cover most of the areas. 
    2.1. If the generated map showed in Rviz isn't good enough, adjusts gmapping params in `/home/robond/catkin_ws/src/rse-nd-home-service-robot/renato_robot/launch/slam_gmapping.launch` and restaart this process.
    2.2. If the generated map showed in Rviz is good enough, then go to `rse-nd-home-service-robot/map` and save your map by running in a new terminal: `rosrun map_server map_saver -f restaurant_renato`. 

- Test your SLAM map and navigate through it using `2D NAV Goal`:
  1. Go to `rse-nd-home-service-robot/scripts` and run `./test_amcl.sh`.
  2. In Rviz, click on `2D NAV Goal` and select a point in the map with cursor to make the robot navigate to it.

- Autonomous navigation through predefined **`Pickup`** and **`DropOff`** goal positions in the map:
  1. Go to `rse-nd-home-service-robot/scripts` and run `./pick_objects.sh`.

- Marker test on the map. It will place a marker in the map on Rviz at **`Pickup`** goal positions, wait 5 seconds, remove marker, wait more 5 seconds and palce the marker at **`DropOff`** goal positions. To avoid write a new c++ script and modify the logic behind **pick_objects** and **add_markers** interactions, a ros command line service call were used on this script.
  1. Go to `rse-nd-home-service-robot/scripts` and run `./add_markers.sh`.
  
- Run the full Home Service Robot. The robot will go to the **`Pickup`** goal position, simulate a pickup action during 5 seconds, go to **`DropOff`** goal position and simulate a drop off action.
  1. Go to `rse-nd-home-service-robot/scripts` and run `./home_service.sh`.

## License
The contents of this repository are covered under the MIT License.
