#ifndef CATKIN_WS_ROBOTBEHAVIORALGORITHMTELEOP_H
#define CATKIN_WS_ROBOTBEHAVIORALGORITHMTELEOP_H

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <map>
#include <vector>
#include <termios.h>
#include <sensor_msgs/LaserScan.h>
#include "RobotMoveActions.h"

class RobotBehaviorAlgorithmTeleop: public RobotMoveActions{
public:
    // #######################################################################################
 	  // @.@ Constructor
    // #######################################################################################
    RobotBehaviorAlgorithmTeleop(float s, float t) : RobotMoveActions(s,t){}
    
    // #######################################################################################
 	  // @.@ Helper member functions public
 	  // #######################################################################################

    // +.+ before teleop starts display helper message
    void displayMessageTeleop();

    // +.+ get the twist value from teleop actions
    geometry_msgs::Twist getTwistTeleop();

private:

    // #######################################################################################
 	  // @.@ Helper member functions private
 	  // #######################################################################################

    // calculate the twist(velocity commands) to send to the robot
    void calculateTwistTeleop();

    // For non-blocking keyboard inputs, get keyboard inpute
    int keyboard_ch(void);

   
    // #######################################################################################
 	  // @.@ Handy Variables
    // #######################################################################################
   
    struct termios oldt_;
    struct termios newt_;
    int ch_;
    char key_{' '};


    // +.+ key map for teleop
    std::map<char, std::vector<float>> move_bindings_
    {
      {'w', move_actions["forward"]},
      {'x', move_actions["backward"]},
      {'a', move_actions["left"]},
      {'d', move_actions["right"]},
      {'s', move_actions["stop"]}
    };

    // +.+ increase or decrease the speed values
    std::map<char, std::vector<float>> speed_bindings_
    {
      {'o', {1.1, 1.1}},
      {'p', {0.9, 0.9}},
      {'j', {1.1, 1}},
      {'k', {0.9, 1}},
      {'n', {1, 1.1}},
      {'m', {1, 0.9}}
    };


    // Reminder message
    const char* helper_teleop_ = R"(
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
)";

};
#endif

