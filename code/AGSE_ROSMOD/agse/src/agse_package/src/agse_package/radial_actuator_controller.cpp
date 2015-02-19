#include "agse_package/radial_actuator_controller.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void radial_actuator_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;

  // THESE NEED TO BE UPDATED
  epsilon = 5;
  motorForwardPin = 57;
  motorBackwardPin = 58;

  // set up the pins to control the h-bridge
  gpio_export(motorForwardPin);
  gpio_export(motorBackwardPin);
  gpio_set_dir(motorForwardPin,OUTPUT_PIN);
  gpio_set_dir(motorBackwardPin,OUTPUT_PIN);
  // set up the encoder module
  rm_eqep_period = 1000000000L;
  radialMotoreQEP.initialize("/sys/devices/ocp.3/48302000.epwmss/48302180.eqep", eQEP::eQEP_Mode_Absolute);
  radialMotoreQEP.set_period(rm_eqep_period);
  // initialize the goal position to 0
  radialGoal = 0;
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void radial_actuator_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Radial motor PAUSED!" : "Radial motor UNPAUSED!");
}
//# End controlInputs_sub_OnOneData Marker

// Component Service Callback
//# Start radialPosCallback Marker
bool radial_actuator_controller::radialPosCallback(agse_package::radialPos::Request  &req,
    agse_package::radialPos::Response &res)
{
    // Business Logic for radialPos_server Server providing radialPos Service
  if (req.update == true)
    {
      radialGoal = req.goal;
    }
  res.current = radialCurrent;
  return true;
}
//# End radialPosCallback Marker

// Callback for radialPosTimer timer
//# Start radialPosTimerCallback Marker
void radial_actuator_controller::radialPosTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for radialPosTimer 
  if (!paused)
    {
      // read current value for radial position (encoder)
      radialCurrent = radialMotoreQEP.get_position();
      // update motor based on current value
      if ( abs(radialGoal-radialCurrent) > epsilon ) // if there's significant reason to move
	{
	  if (radialGoal > radialCurrent) 
	    {
	      gpio_set_value(motorBackwardPin,LOW);
	      gpio_set_value(motorForwardPin,HIGH);
	    }
	  else
	    {
	      gpio_set_value(motorForwardPin,LOW);
	      gpio_set_value(motorBackwardPin,HIGH);
	    }
	}
    }
}
//# End radialPosTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
radial_actuator_controller::~radial_actuator_controller()
{
    radialPosTimer.stop();
    controlInputs_sub.shutdown();
    radialPos_server.shutdown();
}

void radial_actuator_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&radial_actuator_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: radialPos_server
    ros::AdvertiseServiceOptions radialPos_server_options;
    radialPos_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::radialPos>
	    ("radialPos",
             boost::bind(&radial_actuator_controller::radialPosCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->radialPos_server = nh.advertiseService(radialPos_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&radial_actuator_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.01),
	     boost::bind(&radial_actuator_controller::radialPosTimerCallback, this, _1),
	     &this->compQueue);
    this->radialPosTimer = nh.createTimer(timer_options);

}
