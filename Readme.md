# Explore Turtlebot World 

## Description of the project

This project falls into the application *ROS Node simulated in Gazebo*, as it is one of the suggestions in *Capstone Intro*. The idea behind the project, was to make a ROS Node that will allow a user to explore the Turtlebot World using the Turtlebot robot in an autonomous or manual way. For this purpose the user has 3 available behaviors: 
* **wander**: The robot explores the world randomly and in an autnomous way. In this mode, the *Turtlebot* uses the information of the laser scan to avoid collision with obstacles.
* **teleop**: Turtlebot is contolled by the user from keyboard keys. There is no security measures regarding collisions.
* **stop**: The robot will stop any moving action.
This node with some tunning can be used for making a map of an indoor environment.
---
## Requirements
* Linux Ubuntu >= 16.04
* ROS : distribution >= Kinectic 
* Gazebo
* Turtlebot packages
---
## Instructions for building/running the project

### Preparing the system

* ROS installation: please see this [tutorials](https://www.ros.org/install/) and install the desktop full version.
* Gazebo is installed alongside ROS.
* Turtlebot packages: run the following command, if you get problems please check this [link](https://answers.ros.org/question/246015/installing-turtlebot-on-ros-kinetic/).

```
sudo apt install ros-<rosdistro>-turtlebot-*
```
* [Terminator](http://www.linuxandubuntu.com/home/terminator-a-linux-terminal-emulator-with-multiple-terminals-in-one-window) installation is recommended:
```
sudo apt install terminator
```

### Building the project
1. Dowload and unzip the folder
2. In terminal navigate to the project folder
3. Execute the following commands:
```
cd catkin_ws
catkin_make
source devel/setup.bash
```
4. Please do the last command everytime you open a terminal. Notice that you need to be in the catkin_ws folder.

### Running the project
1. In one terminal launch turtlebot world
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
2. In another terminal launch the developed node
```
roslaunch robot_behavior robot_behavior.launch
```
3. Finally from another terminal choose a behavior
```
wander: rostopic pub /robot_bevior std_msgs/String "data: 'wander'"
teleop: rostopic pub /robot_bevior std_msgs/String "data: 'teleop'"
stop: rostopic pub /robot_bevior std_msgs/String "data: 'stop'"
```

### Teleop Behavior Details
If you choose **teleop**, in the terminal from the second option will appear the next message:
```
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w    
   a    s    d
        x    
o/p : increase/decrease max speeds by 10%
j/k : increase/decrease only linear speed by 10%
n/m : increase/decrease only angular speed by 10%
rostopic pub /robot_behavior std_msgs/String "data: 'stop'" for exit
```

To control the robot, select that terminal window and start moving the robot. The keybings are:
* w: move forward
* x: move backwards
* a: rotate left
* d: rotate right
* s: stop

For exiting this mode, please write in the third terminal the command:
```
rostopic pub /robot_bevior std_msgs/String "data: 'stop'"
```

### Wander Behavior Details
In **wander** behavior you can increase or decrease the velocity of the robot with the command bellow:

```
increase 2 times the speed:
rostopic pub /speed_gain std_msgs/Float32 "data: 2.0"

decrease 2 times the speed:
rostopic pub /speed_gain std_msgs/Float32 "data: 0.5"
```

For exiting this mode, please write in the third terminal the command:
```
rostopic pub /robot_bevior std_msgs/String "data: 'stop'"
```
---
## Package Structure

### Package Tree
```
.
├── CMakeLists.txt
├── config
│   └── config_robot_behavior.yaml
├── doc
│   ├── README.md
│   └── RobotBehaviorNode.png
├── include
│   ├── robot_behavior_algorithms
│   │   ├── RobotBehaviorAlgorithmTeleop.h
│   │   ├── RobotBehaviorAlgorithmWander.h
│   │   └── RobotMoveActions.h
│   └── robot_behavior_ros
│       └── RobotBehaviorNode.h
├── launch
│   └── robot_behavior.launch
├── package.xml
└── src
    ├── robot_behavior_algorithms
    │   ├── RobotBehaviorAlgorithmTeleop.cpp
    │   ├── RobotBehaviorAlgorithmWander.cpp
    │   └── RobotMoveActions.cpp
    └── robot_behavior_ros
        └── RobotBehaviorNode.cpp

9 directories, 14 files
```
Notice that the implementation of the Node is separated from the implementation of the behaviors.

### File Description

Files | Description | 
--- | --- |
`CMakeLists.txt` | is the input to the CMake build system for building software packages |
`config_robot_behavior.yaml` | parameters to be loaded by the node |
`doc/README.md` | diagram with the inputs(parameters and subscribers) and outputs(pusblishers) of the node |
`RobotBehaviorAlgorithmTeleop.h` | header file of the class Teleop behavior |
`RobotBehaviorAlgorithmWander.h` | header file of the class Wander behavior  |
`RobotMoveActions.h` | header file with basic actions of the robot. This is a parent class of the behavior classes  |
`RobotBehaviorNode.h` | header file of ros node |
`robot_behavior.launch` | used to start the node  |
`package.xml` | defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other catkin packages |
`RobotBehaviorAlgorithmTeleop.cpp` | Implementation of Teleop Behavior class | 
`RobotBehaviorAlgorithmWander.cpp` | Implementation of Wander class | 
`RobotMoveActions.cpp` | Defines basic mpvement actions and gives the velocities values to be passed to the robot  | 
`RobotBehaviorNode.cpp` | ros node class implementation | 

---
## Rubric points

### Loops, Functions, I/O
Rubric | Where | 
--- | --- | --- |
`The project demonstrates an understanding of C++ functions and control structures` | :heavy_check_mark: spread in the code |
`The project reads data from a file and process the data, or the program writes data to a file.` | :heavy_check_mark: reads parameters from a yaml file. Not sure if this counts. Lines 152-167 -> RobotBehaviorNode.cpp |
`The project accepts user input and processes the input.` | :heavy_check_mark: RobotBehaviorAlgorithmTeleop.cpp  |

### Object Oriented Programming

Rubric | Where | 
--- | --- |
`The project uses Object Oriented Programming techniques.` | :heavy_check_mark: RobotBehaviorNode.cpp/h for example |
`Classes use appropriate access specifiers for class members.` | :heavy_check_mark: RobotBehaviorNode.cpp/h for example |
`Class constructors utilize member initialization lists.` | :heavy_check_mark: I think all the 4 classes |
`Classes abstract implementation details from their interfaces.` | :heavy_check_mark: RobotBehaviorNode.cpp/h for example |
`Classes encapsulate behavior.` | :heavy_check_mark: RobotBehaviorNode.cpp/h for example |
`Classes follow an appropriate inheritance hierarchy.` | :heavy_check_mark::heavy_exclamation_mark: RobotMoveActions is parent class of RobotBehaviorAlgorithmTeleop and RobotBehaviorAlgorithmWander, not sure if it is in the most elegant way |
`Overloaded functions allow the same function to operate on different parameters.` | :heavy_check_mark: check RobotBehavior.cpp. Lines 96 and 155 |
`Derived class functions override virtual base class functions.` | :x: |
`Templates generalize functions in the project.` | :heavy_check_mark: lines 42 and 43 RobotBehaviorNode.h |

### Memory Management

Rubric | Where | 
--- | --- | 
`The project makes use of references in function declarations.` | :heavy_check_mark: RobotBehaviorNode.cpp/h for example and Lines 21 and 35 of RobotBehaviorAlgorithmWander.h |

