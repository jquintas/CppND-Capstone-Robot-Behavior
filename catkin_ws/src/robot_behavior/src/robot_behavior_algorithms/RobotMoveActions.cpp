#include "RobotMoveActions.h"


// #######################################################################################
// @.@ Calculates the values of the twist message to be sent to the robot
// #######################################################################################

void RobotMoveActions::moveActionRobot(std::vector<float> const& robot_action){

twist_.linear.x = robot_action[0] * speed;
twist_.angular.z = robot_action[1] * turn;

}

// #######################################################################################
// @.@ Updates speed and turn vel values
// #######################################################################################

void RobotMoveActions::updateSpeeds(float linear_speed, float turn_spped){

    speed = linear_speed;
    turn = turn_spped;

}
