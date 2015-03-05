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

bool arm_controller::CheckGoals()
{
  if ( abs(goalRadialPos - currentRadialPos) > radialEpsilon )
    return false;
  if ( abs(goalVerticalPos - currentVerticalPos) > verticalEpsilon )
    return false;
  if ( abs(goalArmRotation - currentArmRotation) > armRotationEpsilon )
    return false;
  if ( abs(goalGripperRotation - currentGripperRotation) > gripperRotationEpsilon )
    return false;
  if ( abs(goalGripperPos - currentGripperPos) > gripperPosEpsilon )
    return false;
  return true;
}

void arm_controller::Init_StateFunc()
{
  // Whatever should be here, not quite sure if this is needed.
  // perhaps do calibration (hit limit switches) of linear actuators
  currentState = FINDING_PB;
}

void arm_controller::Finding_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  static float initRadialPos       = 1.0f;
  static float initVerticalPos     = 1.0f;
  static float initArmRotation     = 1.0f;
  static float initGripperRotation = 1.0f;
  static float initGripperPos      = 1.0f;

  static float maxSearchTime = 300.0f; // seconds we are allowed to search

  static float armRotationStep = 30.0f; // degrees between steps of the state search

  static float centerPayloadBayPositionRadius = 10.0f; // once center of PB is in this radius, we are done
  
  static bool foundPB = false;
  static agse_package::payloadBayState internalPBState; // used within this state; global state set when done
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * possibly calibration data, but that's it.

  // State logic:
  // go to max height & median radius & 0 degrees
  // while !foundPB and internalPBState.pos.{r,z} not within radius
  //   get image Processor result
  //   if result has no detection:
  //     if never found PB:
  //       increment arm rotation by arm rotation step
  //     else if found previously:
  //       go half way between previous find location and current location
  //   else if result has detection:
  //     move to detected position (i.e. set goals to detected position)
  // once done with while, set component's PBState and currentState
  // transition to next state (OPENING_PB)
}

void arm_controller::Opening_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * known PB position
  // * directly above PB

  // State logic:
  // send the command to the PB through UART to open the PB,
  // OPTIONAL : use image based detection to confirm PB opens
  // transition to next state (FINDING_SAMPLE) if PB responds well
}

void arm_controller::Finding_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  static float initRadialPos       = 1.0f;
  static float initVerticalPos     = 1.0f;
  static float initArmRotation     = 1.0f;
  static float initGripperRotation = 1.0f;
  static float initGripperPos      = 1.0f;

  static float maxSearchTime = 300.0f; // seconds we are allowed to search

  static float armRotationStep = 30.0f; // degrees between steps of the state search

  static float centerSamplePositionRadius = 10.0f; // once center of sample is in this radius, we are done
  
  static bool foundSample = false;
  static agse_package::sampleState internalSampleState; // used within this state; global state set when done
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * known payload bay position and orientation (not relevant to this state)

  // State logic:
  // go to max height & median radius & 0 degrees
  // while !foundSample and internalSampleState.pos.{r,z} not within radius
  //   get image Processor result
  //   if result has no detection:
  //     if never found sample:
  //       increment arm rotation by arm rotation step
  //     else if found previously:
  //       go half way between previous find location and current location
  //   else if result has detection:
  //     move to detected position (i.e. set goals to detected position)
  // once done with while, set component's sampleState and currentState
  // transition to next state (GRABBING_SAMPLE)
}

void arm_controller::Grabbing_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * known sample position
  // * directly above sample

  // State logic:
  // Orient gripper to sample (based on sample.orientation.theta)
  // Go down to proper Z level for the sample (HOW DO WE DETERMINE THIS)
  // close gripper
  // move up some amount
  // transition to next state (CARRYING_SAMPLE)
}

void arm_controller::Carrying_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * already have the sample
  // * already know the position & orientation of the PB

  // State Logic:
  // move up to max height (to not hit anything when rotating
  // move R,Theta to payloadBay's R,Theta
  // change gripper rotation to payloadBay's orientation (payloadBay.orientation.theta)
  // move down in Z to proper height for PB (HOW DO WE DETERMINE THIS? FROM MARKERS?)
  // transition to next state (INSERTING_SAMPLE)
}

void arm_controller::Inserting_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * payload is in gripper
  // * directly above PB at appropriate height and orientation

  // State Logic:
  // Open Gripper (set gripper pos to open pos)
  // transition to next state (CLOSING_PB)
}

void arm_controller::Closing_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * sample is in PB
  // * directly above PB

  // State logic:
  // move up some amount (to max height)
  // send the command to the PB through UART to close the PB,
  // OPTIONAL : use image based detection to confirm PB closes
  // transition to next state (MOVING_AWAY) if PB responds well
}

void arm_controller::Moving_Away_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * sample is in PB
  // * PB is closed
  // * directly above PB at max height

  // State logic:
  // determine safe zone for arm (based on PB position)
  // move to safe zone
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

  // need to properly initialize the current sensor readings
  currentRadialPos        = -1.0f;
  currentVerticalPos      = -1.0f;
  currentArmRotation      = -1.0f;
  currentGripperRotation  = -1.0f;
  currentGripperPos       = -1.0f;

  // need to properly initialize the current actuator goals
  goalRadialPos        = -1.0f;
  goalVerticalPos      = -1.0f;
  goalArmRotation      = -1.0f;
  goalGripperRotation  = -1.0f;
  goalGripperPos       = -1.0f;

  // need to properly initialize the sample and payloadBay
  sample.pos.r     = -1.0f;
  sample.pos.theta = -1.0f;
  sample.pos.z     = -1.0f;
  sample.orientation.theta = -1.0f;
  sample.orientation.phi   = -1.0f;

  payloadBay.pos.r     = -1.0f;
  payloadBay.pos.theta = -1.0f;
  payloadBay.pos.z     = -1.0f;
  payloadBay.orientation.theta = -1.0f;
  payloadBay.orientation.phi   = -1.0f;

  // need to initialize the epsilons for the goal/current feedback loops
  radialEpsilon          = -1.0f;
  verticalEpsilon        = -1.0f;
  armRotationEpsilon     = -1.0f;
  gripperRotationEpsilon = -1.0f;
  gripperPosEpsilon      = -1.0f;
  // need to initialize the offsets with measurements from the system
  radialOffset          = -1.0f;
  verticalOffset        = -1.0f;
  armRotationOffset     = -1.0f;
  gripperRotationOffset = -1.0f;
  gripperPosOffset      = -1.0f;

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
      // If we haven't gotten where the state said we should go, return
      if ( !CheckGoals() )
	return;
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
	default:
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
    payloadBayStateFromImage_client.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void arm_controller::startUp()
{
    ros::NodeHandle nh;

    // Need to read in and parse the group configuration xml if it exists
    GroupXMLParser groupParser;
    std::string configFileName = nodeName + "." + compName + ".xml";
    if ( boost::filesystem::exists(configFileName) )
    {
        groupParser.Parse(configFileName);
	groupParser.Print();
    }

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
    // client: payloadBayStateFromImage_client
    this->payloadBayStateFromImage_client = nh.serviceClient<agse_package::payloadBayStateFromImage>
	("payloadBayStateFromImage"); 

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
