#include "agse_package/arm_controller.hpp"

//# Start User Globals Marker

void arm_controller::UpdateSensorData()
{
  // all servers have 
  // inputs: int64 goal, bool update
  // output: current
  agse_package::radialPos rPos;
  rPos.request.update = false;
  radialPos_client.call(rPos);
  currentRadialPos = rPos.response.current;

  agse_package::verticalPos vPos;
  vPos.request.update = false;
  verticalPos_client.call(vPos);
  currentVerticalPos = vPos.response.current;

  agse_package::armRotation arm;
  arm.request.update = false;
  armRotation_client.call(arm);
  currentArmRotation = arm.response.current;

  agse_package::gripperRotation gRot;
  gRot.request.update = false;
  gripperRotation_client.call(gRot);
  currentGripperRotation = gRot.response.current;
  
  agse_package::gripperPos gPos;
  gPos.request.update = false;
  gripperPos_client.call(gPos);
  currentGripperPos = gPos.response.current;
}

void arm_controller::UpdateArmPosition()
{
  // all servers have 
  // inputs: int64 goal, bool update
  // output: current
  agse_package::radialPos rPos;
  rPos.request.update = true;
  rPos.request.goal = goalRadialPos;
  radialPos_client.call(rPos);
  currentRadialPos = rPos.response.current;

  agse_package::verticalPos vPos;
  vPos.request.update = true;
  vPos.request.goal = goalVerticalPos;
  verticalPos_client.call(vPos);
  currentVerticalPos = vPos.response.current;

  agse_package::armRotation arm;
  arm.request.update = true;
  arm.request.goal = goalArmRotation;
  armRotation_client.call(arm);
  currentArmRotation = arm.response.current;

  agse_package::gripperRotation gRot;
  gRot.request.update = true;
  gRot.request.goal = goalGripperRotation;
  gripperRotation_client.call(gRot);
  currentGripperRotation = gRot.response.current;
  
  agse_package::gripperPos gPos;
  gPos.request.update = true;
  gPos.request.goal = goalGripperPos;
  gripperPos_client.call(gPos);
  currentGripperPos = gPos.response.current;
}

void arm_controller::Init_StateFunc()
{
  // Whatever should be here, not quite sure if this is needed.
}

void arm_controller::Finding_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Opening_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Finding_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Grabbing_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Carrying_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Inserting_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Closing_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

void arm_controller::Moving_Away_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
}

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void arm_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;
  currentState = INIT;
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void arm_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Arm controller PAUSED!" : "Arm controller UNPAUSED!");
}
//# End controlInputs_sub_OnOneData Marker

// Callback for armTimer timer
//# Start armTimerCallback Marker
void arm_controller::armTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for armTimer 
  if (!paused)
    {
      UpdateSensorData();
      switch (currentState)
	{
	case INIT:
	  Init_StateFunc();
	  break;
	case FINDING_PB:
	  Finding_PB_StateFunc();
	  break;
	case OPENING_PB:
	  Opening_PB_StateFunc();
	  break;
	case FINDING_SAMPLE:
	  Finding_Sample_StateFunc();
	  break;
	case GRABBING_SAMPLE:
	  Grabbing_Sample_StateFunc();
	  break;
	case CARRYING_SAMPLE:
	  Carrying_Sample_StateFunc();
	  break;
	case INSERTING_SAMPLE:
	  Inserting_Sample_StateFunc();
	  break;
	case CLOSING_PB:
	  Closing_PB_StateFunc();
	  break;
	case MOVING_AWAY:
	  Moving_Away_StateFunc();
	  break;
	}
      UpdateArmPosition();
    }
}
//# End armTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
arm_controller::~arm_controller()
{
    armTimer.stop();
    controlInputs_sub.shutdown();
    sampleStateFromImage_client.shutdown();
    radialPos_client.shutdown();
    armRotation_client.shutdown();
    gripperRotation_client.shutdown();
    verticalPos_client.shutdown();
    gripperPos_client.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void arm_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&arm_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all required services associated with this component
    // client: sampleStateFromImage_client
    this->sampleStateFromImage_client = nh.serviceClient<agse_package::sampleStateFromImage>
	("sampleStateFromImage"); 
    // client: radialPos_client
    this->radialPos_client = nh.serviceClient<agse_package::radialPos>
	("radialPos"); 
    // client: armRotation_client
    this->armRotation_client = nh.serviceClient<agse_package::armRotation>
	("armRotation"); 
    // client: gripperRotation_client
    this->gripperRotation_client = nh.serviceClient<agse_package::gripperRotation>
	("gripperRotation"); 
    // client: verticalPos_client
    this->verticalPos_client = nh.serviceClient<agse_package::verticalPos>
	("verticalPos"); 
    // client: gripperPos_client
    this->gripperPos_client = nh.serviceClient<agse_package::gripperPos>
	("gripperPos"); 

    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&arm_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.02),
	     boost::bind(&arm_controller::armTimerCallback, this, _1),
	     &this->compQueue);
    this->armTimer = nh.createTimer(timer_options);

}
