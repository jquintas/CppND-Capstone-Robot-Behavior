#ifndef CATKIN_WS_ROBOTBEHAVIORALGORITHMWANDER_H
#define CATKIN_WS_ROBOTBEHAVIORALGORITHMWANDER_H

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Time.h>
#include "RobotMoveActions.h"

class RobotBehaviorAlgorithmWander: public RobotMoveActions{
public:

    // #######################################################################################
 	// @.@ Constructor
    // #######################################################################################
     RobotBehaviorAlgorithmWander(float s, float t) : RobotMoveActions(s,t) {}

    // #######################################################################################
 	// @.@ Helper member functions public
 	// #######################################################################################

    // +.+ Get the middle range
    float getRangeAhead(sensor_msgs::LaserScan const &laser) const;

    // +.+ get the current speed gain
    int getSpeedGain() const;

    // +.+ set a new speed gain
    void setNewSpeedGain(const float gain);

    // +.+ set the initial time for wander behavior
    void setStarterTimer();

    geometry_msgs::Twist getTwistWander(const float range_ahead);
    
    // +.+ Calculates the desired speed to send to the robot
    void moveActionRobot(std::vector<float> const& robot_action, int speed_gain);

private:
    // #######################################################################################
 	// @.@ Helper member functions private
 	// #######################################################################################

    // +.+ Where the wander behavior happens
    void twistWander(const float range_ahead);
    
    
    // #######################################################################################
 	// @.@ Handy Variables
    // #######################################################################################

    // +.+ Time between actions
    double action_change_time_;
    
    float speed_gain_{1};

    // +.+ Boolean to control if the robot moves forward or another action 
    bool going_forward_{true};

};
#endif

