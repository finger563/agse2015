#include "agse_package/user_input_controller.hpp"

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

int timerDelay = 500;
int timerIterations = 0;
bool paused = true;

// Init Function
void user_input_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
    // Stop Init Timer
    initOneShotTimer.stop();
}

// Callback for userInputTimer timer
void user_input_controller::userInputTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for userInputTimer 
  timerIterations++;
  if (timerIterations >= timerDelay){
    timerIterations = 0;
    paused = !paused;
    agse_package::controlInputs control;
    control.paused = paused;
    controlInputs_pub.publish(control);
    if (paused)
      ROS_INFO("Pausing the system!");
    else
      ROS_INFO("Unpausing the system!");
  }
}

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
user_input_controller::~user_input_controller()
{
    userInputTimer.stop();
    controlInputs_pub.shutdown();
}

void user_input_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all publishers associated with this component
    // publisher: controlInputs_pub
    this->controlInputs_pub = nh.advertise<agse_package::controlInputs>
	("controlInputs", 1000);	

    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&user_input_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.name
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&user_input_controller::userInputTimerCallback, this, _1),
	     &this->compQueue);
    this->userInputTimer = nh.createTimer(timer_options);

}
