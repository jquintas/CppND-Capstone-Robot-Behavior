/*
Implementation of wander behavior methods
*/
#include "RobotBehaviorAlgorithmWander.h"


 /*
#######################################################################################################################
@.@ Get the middle range
#######################################################################################################################
*/
float RobotBehaviorAlgorithmWander::getRangeAhead(const sensor_msgs::LaserScan &laser) const{
    return laser.ranges[laser.ranges.size()/2 -1];
}


 /*
#######################################################################################################################
@.@ Set a new speed gain
#######################################################################################################################
*/
int RobotBehaviorAlgorithmWander::getSpeedGain() const{
    return speed_gain_;
}


 /*
#######################################################################################################################
@.@ Set new speed gain 
#######################################################################################################################
*/
void RobotBehaviorAlgorithmWander::setNewSpeedGain(const float gain){
    speed_gain_ = gain;
}


 /*
#######################################################################################################################
@.@ Starter timer for wander behavior
#######################################################################################################################
*/
void RobotBehaviorAlgorithmWander::setStarterTimer(){
    action_change_time_ = ros::Time::now().toSec();
}



 /*
#######################################################################################################################
 @.@ Calculate the twist values resulting from wander movements
 #######################################################################################################################
 */
void RobotBehaviorAlgorithmWander::moveActionRobot(std::vector<float> const& robot_action, int speed_gain){
    twist_.linear.x = robot_action[0] * speed * speed_gain;
    twist_.angular.z = robot_action[1] * turn * speed_gain;
}


 /*
#######################################################################################################################
 @.@ Get twist values from wander behavior
 #######################################################################################################################
 */
geometry_msgs::Twist RobotBehaviorAlgorithmWander::getTwistWander(const float range_ahead){
    twistWander(range_ahead);
    return twist_;
}

 /*
#######################################################################################################################
 @.@ Wander Behavior 
 #######################################################################################################################
 */
void RobotBehaviorAlgorithmWander::twistWander(const float range_ahead){
    if (going_forward_){
        // +.+ In case of going forward check if the laserscan reading is below 2 or if it passed a period of time. 
        // Move forward will be switched for a rotating action for 4 seconds.
        if (range_ahead < 2 || ros::Time::now().toSec() > action_change_time_){
            going_forward_ = false;
            action_change_time_ = ros::Time::now().toSec() + ros::Duration(4).toSec();
        }
    }
    else{  // +.+ when robot is rotating, just chek if the time duration passed. If the duration is reached, the action of rotating 
           // the robot is switched to forward. The robot now will move 5 seconds forward unless an obstacle appears before. 
        if (ros::Time::now().toSec() > action_change_time_){
            going_forward_ = true;
            action_change_time_ = ros::Time::now().toSec() + ros::Duration(5).toSec();
        }

    }

    if (going_forward_){
        moveActionRobot(move_actions["forward"], speed_gain_);
    }
    else{
        moveActionRobot(move_actions["left"], speed_gain_);
        //auto it = move_actions.begin();
        // Tried Random but doesn't work well
        //std::advance(it, rand() % move_actions.size()-1);
        //move_robot(move_actions[it->first], speed_gain_);
        //std::cout << "actions" << it->first << '\n';
    }

}       

