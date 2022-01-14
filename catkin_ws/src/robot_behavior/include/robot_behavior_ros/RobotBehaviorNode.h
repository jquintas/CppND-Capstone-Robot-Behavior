#ifndef CATKIN_WS_ROBOTBEHAVIORNODE_H
#define CATKIN_WS_ROBOTBEHAVIORNODE_H

//some generically useful stuff to include...
#include <stdlib.h>
#include <ros/ros.h> //ALWAYS need to include this 

// ROS messages and stuff
// Examples
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "RobotBehaviorAlgorithmTeleop.h"
#include "RobotBehaviorAlgorithmWander.h"


// +.+ Robot Behaviors 
 enum Behaviors {Wander, Teleop, Stop};

 class RobotBehaviorNode {
 public:
 	// #############################
 	// @.@ Constructor
 	// #############################
 	// "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
 	RobotBehaviorNode(ros::NodeHandle &&nodehandle);

 	// #############################
 	// @.@ Destructor
 	// #############################
 	~RobotBehaviorNode();

 	// #############################
 	// @.@ Public methods
 	// #############################
 	int nodeFrequency();
	void displayHelperMessage();
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name);
	template <typename T> T getParameters(ros::NodeHandle &_nh, std::string const &parameter_name, T default_value);


 private:
 	// put private member data here; "private" data will only be available to member functions of this class;
 	ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

 	// some objects to support subscriber and publishers, these will be set up within the class constructor, hiding the ugly details

 	// #####################
 	// @.@ Subsctibers
 	// #####################

	// +.+ Subscribes to the beahavior message: "wander", "teleop", "stop" 
 	ros::Subscriber explore_behavior_sub_;
	
	// +.+ Gets the values from laser
	ros::Subscriber laser_scan_sub_;

	// +.+ allows to adjust the speed velocity in wander behavior
	ros::Subscriber speed_gain_sub_;

 	// #####################
 	// @.@ Publishers
 	// #####################

	// +.+ publishes the velocities to the robot
 	ros::Publisher cmd_vel_pub_;

	// #######################
 	// @.@ Timer
 	// #######################
 	ros::Timer timer_;

 	// ####################################################################################################################
 	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
 	// member variables will retain their values even as callbacks come and go
 	// ####################################################################################################################

 	// Parameters from config Yaml
 	// always p_paraName -> Example: double p_ku_;
	int p_node_frequency_;
	float p_linear_speed_{1}, p_turn_speed_{1};

 	// Problem variables
	geometry_msgs::Twist twist_to_robot_;
	float laser_range_{0};

	 // +.+ Helper message
	const char* msg_helper_ = R"(
Explore autonomous or manually the turtlebot world
autonomous (wander) : rostopic pub /robot_behavior "data:= 'wander'"
manually (teleop): rostopic pub /robot_behavior	"data:= 'teleop'"
stop: rostopic pub /robot_behavior "data:= 'stop'"
CTRL-C to quit
)";

	// +.+ Default behavior
	Behaviors robot_behavior_ = Behaviors::Stop;

	// Initializes the classes responsible for teleop and wander behaviors 
	RobotBehaviorAlgorithmTeleop robot_teleop_ = RobotBehaviorAlgorithmTeleop(p_linear_speed_, p_turn_speed_); 
	RobotBehaviorAlgorithmWander robot_wander_ = RobotBehaviorAlgorithmWander(p_linear_speed_, p_turn_speed_);
 	
	 // #######################################################################################
 	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	// #######################################################################################
 	void initializeSubscribers();
 	void initializePublishers();
 	void initializeTimer();
 	void loadParams();


 	// #######################################################################################
 	// @.@ Callbacks declaration
 	// #######################################################################################
 	void timerIterCallback(const ros::TimerEvent& event);
	void exploreBehaviorCallback(const std_msgs::String& msg);
	void laserScanCallback(const sensor_msgs::LaserScan& msg);
	void speedCallback(const std_msgs::Float32& msg);


	// #######################################################################################
 	// @.@ Private helper member functions
 	// #######################################################################################
	 void updateSpeedValues();
}; 
#endif 
