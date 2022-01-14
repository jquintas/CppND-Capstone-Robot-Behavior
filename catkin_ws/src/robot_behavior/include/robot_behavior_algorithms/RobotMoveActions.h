#ifndef CATKIN_WS_ROBOTMOVEACTIONS_H
#define CATKIN_WS_ROBOTMOVEACTIONS_H

#include <vector>
#include <string>
#include <geometry_msgs/Twist.h>

class RobotMoveActions{
public:
    // #######################################################################################
 	// @.@ Constructor
    // #######################################################################################
 	
    RobotMoveActions(float s, float t) : speed(s), turn(t) {}


    // #######################################################################################
 	// @.@ Handy Variables
 	// #######################################################################################

    float speed;
    float turn;
    
    // +-+ map with basic actions that the robot can do. The final vel is controlled by speed and turn
    std::map<std::string, std::vector<float>> move_actions
    {
        {"forward", {1.0, 0.0}},
        {"backward", {-1.0, 0.0}},
        {"right", {0.0, -1.0}},
        {"left", {0.0, 1.0}},
        {"stop", {0.0, 0.0}}
    };

    // #######################################################################################
 	// @.@ Helper member functions
 	// #######################################################################################
    
    // +.+ Calculates the values of the twist message to be sent to the robot
    void moveActionRobot(std::vector<float> const& robot_action);
    
    // +.+ updates speed and turn vel values
    void updateSpeeds(float linear_speed, float turn_spped);


protected:

    // +.+ message type to send to the robot
    geometry_msgs::Twist twist_;


};
#endif