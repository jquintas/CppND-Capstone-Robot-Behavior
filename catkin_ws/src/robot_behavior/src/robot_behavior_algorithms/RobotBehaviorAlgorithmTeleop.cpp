#include <RobotBehaviorAlgorithmTeleop.h>

 /*
#######################################################################################################################
@.@ Get the middle range
#######################################################################################################################
*/
void RobotBehaviorAlgorithmTeleop::displayMessageTeleop(){
    	printf("%s", helper_teleop_);

}

 /*
#######################################################################################################################
@.@ checks which key was pressed
#######################################################################################################################
*/
int RobotBehaviorAlgorithmTeleop::keyboard_ch()
{
   // +.+ Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt_);
  newt_ = oldt_;

  // +.+ Make required changes and apply the settings
  newt_.c_lflag &= ~(ICANON | ECHO);
  newt_.c_iflag |= IGNBRK;
  newt_.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt_.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt_.c_cc[VMIN] = 1;
  newt_.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt_);

  // +.+ Get the current character
  key_ = getchar();

  // +.+ Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);

}

 /*
#######################################################################################################################
@.@ get twist values from teleop
#######################################################################################################################
*/
geometry_msgs::Twist RobotBehaviorAlgorithmTeleop::getTwistTeleop(){
    calculateTwistTeleop();
    return twist_;
}

 /*
#######################################################################################################################
@.@ Calculate twist according to key pressed
#######################################################################################################################
*/
void RobotBehaviorAlgorithmTeleop::calculateTwistTeleop(){

    keyboard_ch();
    // +.+ If the key corresponds to a key in moveBindings
    if (move_bindings_.count(key_) == 1)
    {
      // +.+ Grab the direction data
      moveActionRobot(move_bindings_[key_]);

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key_);
    }

    // +.+ Otherwise if it corresponds to a key in speedBindings
    else if (speed_bindings_.count(key_) == 1)
    {
      // +.+ Grab the speed data
      speed *= speed_bindings_[key_][0];
      turn *= speed_bindings_[key_][1];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key_);
    }

    // +.+ Otherwise, set the robot to stop
    else
    {
      moveActionRobot(move_bindings_['s']);

      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key_);
    }

}