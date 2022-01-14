/*
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "RobotBehaviorNode"
#include "RobotBehaviorNode.h"

 /*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle rvalue into constructor for constructor to build subscribers, etc
#######################################################################################################################
*/
RobotBehaviorNode::RobotBehaviorNode(ros::NodeHandle &&nodehandle):nh_(nodehandle) {
//RobotBehaviorNode::RobotBehaviorNode(){

 	ROS_INFO("in class constructor of RobotBehaviorNode");
 	initializeSubscribers();
 	initializePublishers();
 	loadParams();
 	initializeTimer();

 }

 /*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
RobotBehaviorNode::~RobotBehaviorNode() {

 	// +.+ shutdown publishers
 	// ---> add publishers here
 	 cmd_vel_pub_.shutdown();

 	// +.+ shutdown subscribers
 	// ---> add subscribers here
 	explore_behavior_sub_.shutdown();
	speed_gain_sub_.shutdown();
	laser_scan_sub_.shutdown();

 	// +.+ stop timer
 	timer_.stop();

 	// +.+ shutdown node
 	nh_.shutdown();
 }

 /*
#######################################################################################################################
@.@ Member Helper function to set up subscribers;
note odd syntax: &RobotBehaviorNode::subscriberCallback is a pointer to a member function of RobotBehaviorNode
"this" keyword is required, to refer to the current instance of RobotBehaviorNode
#######################################################################################################################
*/
void RobotBehaviorNode::initializeSubscribers() {
 	ROS_INFO("Initializing Subscribers for RobotBehaviorNode");
 	// ---> add subscribers here
 	// Example: state_sub = nh_.subscribe("State", 10, &RobotBehaviorNode::updateCallback,this);

 	explore_behavior_sub_ = nh_.subscribe("robot_behavior", 10, &RobotBehaviorNode::exploreBehaviorCallback,this);
	laser_scan_sub_ = nh_.subscribe("scan", 10, &RobotBehaviorNode::laserScanCallback, this);
	speed_gain_sub_ = nh_.subscribe("speed_gain", 10, &RobotBehaviorNode::speedCallback, this);

}

 /*
#######################################################################################################################
@.@ Member helper function to set up publishers;
#######################################################################################################################
*/
void RobotBehaviorNode::initializePublishers() {
	ROS_INFO("Initializing Publishers for RobotBehaviorNode"); 	// ---> add publishers here

	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",10);

}

 /*
#######################################################################################################################
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
#######################################################################################################################
*/
void RobotBehaviorNode::initializeTimer() {
 	timer_ =nh_.createTimer(ros::Duration(1.0/RobotBehaviorNode::nodeFrequency()), &RobotBehaviorNode::timerIterCallback, this);
	// +.+ Display helper message with the behaviors available and how to use them
	displayHelperMessage();
 	timer_.stop();
}

 /*
#######################################################################################################################
@.@ Set frequency of the node frequency
#######################################################################################################################
*/
int RobotBehaviorNode::nodeFrequency()
{
 	
	p_node_frequency_ = getParameters<int>(nh_, "/RobotBehaviorNode/node_frequency");
	//p_node_frequency_ = getParameters<int>(nh_, "/RobotBehaviorNode/node_frequency", 10);
 	ROS_INFO("Node will run at : %d [hz]", p_node_frequency_);
 	return p_node_frequency_;
}

void RobotBehaviorNode::displayHelperMessage()
{
	printf("%s", msg_helper_);
}

/*
#######################################################################################################################
@.@ Member functions template to read the parameters
#######################################################################################################################
*/

//+.+ Option not considering default value, so the config file must have the parameter
template <typename T> T RobotBehaviorNode::getParameters(ros::NodeHandle &_nh, std::string const &parameter_name){
	
	T parameter;
	
	if (!_nh.getParam(parameter_name, parameter)) {
		ROS_ERROR("No parameter [%s] shutting down", parameter_name.c_str());
		ros::shutdown();
	}
	else{
		_nh.getParam(parameter_name, parameter);
		ROS_INFO_STREAM("Parameter [" << parameter_name.c_str() << "] configured. Using loaded value " << parameter <<"\n");
		return parameter;
	}
}

//+.+ Option considering default value. Even if the parameter doesn't exist in config file it is possible to use a default value.
template <typename T> T RobotBehaviorNode::getParameters(ros::NodeHandle &_nh, std::string const &parameter_name, T default_value){
		T parameter;
	
	if (!_nh.getParam(parameter_name, parameter)) {
		parameter = default_value;
		//ROS_INFO("No parameter [%s] configured, using the default value %s", parameter_name.c_str(),parameter);
		ROS_INFO_STREAM("No parameter [" << parameter_name.c_str() <<"] configured. Using a default value " << parameter << "\n");
		
	}
	else{
		_nh.getParam(parameter_name, parameter);
		ROS_INFO_STREAM("Parameter [" << parameter_name.c_str() << "] configured. Using loaded value " << parameter <<"\n");
	}
	return parameter;
}

 /*
#######################################################################################################################
@.@ Load the parameters
#######################################################################################################################
*/
void RobotBehaviorNode::loadParams() {
 	ROS_INFO("Load the RobotBehaviorNode parameters");
 	//---> params here, always p_paramName
	// +.+ no default value in config file, so it assume 0.5
	p_linear_speed_ = getParameters<float>(nh_, "/RobotBehaviorNode/linear_speed", 0.5);

	// +.+ value present in config file, so it will read and use it (turn_speed: 0.3)
	p_turn_speed_ = getParameters<float>(nh_, "/RobotBehaviorNode/turn_speed", 0.5);

	// +.+ update speed paramter in behaviors classes; maybe there is a smarter way to do this
	robot_teleop_.updateSpeeds(p_linear_speed_, p_turn_speed_);
	robot_wander_.updateSpeeds(p_linear_speed_, p_turn_speed_);

 }


 /*
#######################################################################################################################
@.@ Callbacks Section / Methods
#######################################################################################################################
*/

 /*
#######################################################################################################################
@.@ Iteration via timer callback, where the magic occurs
#######################################################################################################################
*/
void RobotBehaviorNode::timerIterCallback(const ros::TimerEvent &event) {

 	// ####################################################################################################################
 	// @.@ Do your algorithm part and publish things here, like while loop with ros::Rate rate and rate.sleep(duration)
 	// in classical ros
 	// ###################################################################################################################

	switch(robot_behavior_)
	{
		case Behaviors::Wander:
			//ROS_INFO("Wander behavior");
			twist_to_robot_ = robot_wander_.getTwistWander(laser_range_);
			cmd_vel_pub_.publish(twist_to_robot_);
			break;
		case Behaviors::Teleop:
			//ROS_INFO("Teleop behavior");
			twist_to_robot_ = robot_teleop_.getTwistTeleop();
			cmd_vel_pub_.publish(twist_to_robot_);
			break; 
		default:
			ROS_INFO("Robot Stop");
	}	
 }
 /*
#######################################################################################################################
@.@ Callback Behavior: wander, teleop, stop
#######################################################################################################################
*/
void RobotBehaviorNode::exploreBehaviorCallback(const std_msgs::String& msg) {

 	if (msg.data == "wander"){
 		 ROS_INFO("Timer will Start with the behavior %s", msg.data.c_str());
		 robot_behavior_ = Behaviors::Wander; 
		 robot_wander_.setStarterTimer(); // register the time when wander behavior is activated
		 timer_.start();				  // starts iteration via a timer callback
 	} 	else if(msg.data == "teleop"){
		 ROS_INFO("Timer will Start with the behavior %s", msg.data.c_str());
 		 robot_behavior_ = Behaviors::Teleop;
		 robot_teleop_.displayMessageTeleop(); // +.+ display teleop helper message
 		 timer_.start();					   // starts iteration via a timer callback
	} else {
		 ROS_INFO("Timer will Stop");
		 robot_behavior_ = Behaviors::Stop;
 		 timer_.stop();							// stops time so the iteration callback
	}
 
 }

 /*
#######################################################################################################################
@.@ Callback laser ranges -> get laser readings and obtain the middle value
#######################################################################################################################
*/
void RobotBehaviorNode::laserScanCallback(const sensor_msgs::LaserScan& msg){
	laser_range_ =robot_wander_.getRangeAhead(msg);
}

 /*
#######################################################################################################################
@.@ Callback to change the speed gain in wanderbot behavior
#######################################################################################################################
*/
void RobotBehaviorNode::speedCallback(const std_msgs::Float32& msg){
	robot_wander_.setNewSpeedGain(msg.data);
}


 /*
#######################################################################################################################
@.@ Main
#######################################################################################################################
*/
 int main(int argc, char** argv)
 {
 	// +.+ ROS set-ups:
 	ros::init(argc, argv, "robot_behavior_node"); //node name
 	// +.+ create a node handle; need to pass this to the class constructor
	ros::NodeHandle nh;

 	ROS_INFO("main: instantiating an object of type RobotBehaviorNode");

 	// +.+ instantiate an RobotBehaviorNode class object and pass by rvalue to nodehandle for constructor to use
	RobotBehaviorNode robotBehavior(std::move(nh));
	 // +.+ Added to work with timer -> going into spin; let the callbacks do all the work
 	ros::spin();

 	return 0;
 }

