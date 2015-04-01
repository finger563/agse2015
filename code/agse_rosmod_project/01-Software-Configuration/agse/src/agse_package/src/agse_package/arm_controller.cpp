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
  if ( abs(goalRadialPos - currentRadialPos) > radialEpsilon && goalRadialPos >= 0.0f )
    return false;
  if ( abs(goalVerticalPos - currentVerticalPos) > verticalEpsilon && goalVerticalPos >= 0.0f  )
    return false;
  if ( abs(goalArmRotation - currentArmRotation) > armRotationEpsilon && goalArmRotation >= 0.0f )
    return false;
  if ( abs(goalGripperRotation - currentGripperRotation) > gripperRotationEpsilon && goalGripperRotation >= 0.0f )
    return false;
  if ( abs(goalGripperPos - currentGripperPos) > gripperPosEpsilon && goalGripperPos >= 0.0f )
    return false;
  return true;
}

void arm_controller::Init_StateFunc()
{
  // Whatever should be here, not quite sure if this is needed.
  // perhaps do calibration (hit limit switches) of linear actuators
  currentState = FINDING_PB;

  call_at_timer = true;
}

void arm_controller::Finding_PB_StateFunc()
{

  call_at_timer = false;

  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  static float initRadialPos       = (maxRadialPos + minRadialPos) / 2.0f;
  static float initVerticalPos     = maxVerticalPos;
  static float initArmRotation     = minArmRotation;
  static float initGripperRotation = (maxGripperRotation + minGripperRotation) / 2.0f;
  static float initGripperPos      = gripperPosOpened;

  static float maxSearchTime = 300.0f; // seconds we are allowed to search

  static float armRotationStep = 30.0f; // degrees between steps of the state search
  static float radialPosStep = 1.0f;    // amount to move by in radius

  static float positionRadius = 100.0f; // once center of PB is in this radius, we are done
  
  static bool foundPB = false;
  static agse_package::payloadBayState internalPBState; // used within this state; global state set when done
  static float pbX, pbY; // image-space position of Payload Bay

  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * possibly calibration data, but that's it.

  // State logic:
  // if !foundPB or internalPBState.pos.{r,z} not within radius
  if ( !foundPB || abs(pbX) > positionRadius || abs(pbY) > positionRadius )
    {
      // get image Processor result
      bool newTest = false;
      agse_package::payloadBayStateFromImage pbStateImage;
      if ( payloadBayStateFromImage_client.call(pbStateImage) )
	{
	  switch (pbStateImage.response.status)
	    {
	    case DETECTED:
	    case PARTIAL:
	      newTest = true;
	      break;
	    default: // covers the HIDDEN case too, already initialized to false
	      break;
	    }
	  // if result has no detection:
	  if (!newTest)
	    {
	      if (!foundPB) // if never found PB:
		{
		  // increment arm rotation by arm rotation step
		  initArmRotation += armRotationStep;

		  goalRadialPos = initRadialPos;
		  goalVerticalPos = initVerticalPos;
		  goalArmRotation = initArmRotation;
		  goalGripperRotation = initGripperRotation;
		  goalGripperPos = initGripperPos;
		} else // else if found previously:
		{
		  // go half way between previous find location and current location
		  // TODO: THIS MAY NOT FIND THE PB?
		  float previousArmRotation = internalPBState.pos.theta;

		  goalRadialPos = initRadialPos;
		  goalVerticalPos = initVerticalPos;
		  goalArmRotation = (goalArmRotation + previousArmRotation) / 2.0f;
		  goalGripperRotation = initGripperRotation;
		  goalGripperPos = initGripperPos;
		}
	    } else // else if result has detection:
	    {
	      // update internal variables
	      foundPB = true;
	      pbX = pbStateImage.response.x;
	      pbY = pbStateImage.response.y;
	      // if the payload bay's current image-space position is within the allowable radius
	      if ( abs(pbX) <= positionRadius && abs(pbY) <= positionRadius ) 
		{
		  // update the internalPBState
		  internalPBState.pos.r = currentRadialPos;
		  internalPBState.pos.theta = currentArmRotation;
		  internalPBState.pos.z = currentVerticalPos;
		  internalPBState.orientation.theta = pbStateImage.response.angle;
		} else // need to center the payload
		{
		  // move to detected position (i.e. set goals to detected position)
		  // if pbY > 0 : extend radius, else if pbY < 0 retract radius
		  if ( pbY > 0 )
		    {
		      initRadialPos += radialPosStep;
		    } else
		    {
		      initRadialPos -= radialPosStep;
		    }
		  // if pbX > 0 : rotate CW, else if pbX < 0 rotate CCW
		  if ( pbX > 0 )
		    {
		      initArmRotation -= armRotationStep / 2.0f;
		    } else
		    {
		      initArmRotation += armRotationStep / 2.0f;
		    }
		}
	    }
	}
    } else // else payload has been found and is within radius
    {
      // set component's PBState
      payloadBay = internalPBState;
      // transition to next state (OPENING_PB)
      currentState = OPENING_PB;
    }
}

void arm_controller::Opening_PB_StateFunc()
{

  call_at_timer = false;

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

  call_at_timer = false;

  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  static float initRadialPos       = (maxRadialPos + minRadialPos) / 2.0f;
  static float initVerticalPos     = maxVerticalPos;
  static float initArmRotation     = minArmRotation;
  static float initGripperRotation = (maxGripperRotation + minGripperRotation) / 2.0f;
  static float initGripperPos      = gripperPosOpened;

  static float maxSearchTime = 300.0f; // seconds we are allowed to search

  static float armRotationStep = 30.0f; // degrees between steps of the state search
  static float radialPosStep = 1.0f;    // amount to move by in radius

  static float positionRadius = 100.0f; // once center of sample is in this radius, we are done
  
  static bool foundSample = false;
  static agse_package::sampleState internalSampleState; // used within this state; global state set when done
  static float sX, sY; // image-space position of sample

  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary

  // starting with:
  // * known payload bay position and orientation (not relevant to this state)

  // State logic:
  // if !foundSample or internalSampleState.pos.{r,z} not within radius
  if ( !foundSample || abs(sX) > positionRadius || abs(sY) > positionRadius )
    {
      // get image Processor result
      bool newTest = false;
      agse_package::sampleStateFromImage sStateImage;
      if ( payloadBayStateFromImage_client.call(sStateImage) )
	{
	  switch (sStateImage.response.status)
	    {
	    case DETECTED:
	    case PARTIAL:
	      newTest = true;
	      break;
	    default: // covers the HIDDEN case too, already initialized to false
	      break;
	    }
	  // if result has no detection:
	  if (!newTest)
	    {
	      if (!foundSample) // if never found Sample:
		{
		  // increment arm rotation by arm rotation step
		  initArmRotation += armRotationStep;

		  goalRadialPos = initRadialPos;
		  goalVerticalPos = initVerticalPos;
		  goalArmRotation = initArmRotation;
		  goalGripperRotation = initGripperRotation;
		  goalGripperPos = initGripperPos;
		} else // else if found previously:
		{
		  // go half way between previous find location and current location
		  // TODO: THIS MAY NOT FIND THE Sample?
		  float previousArmRotation = internalSampleState.pos.theta;

		  goalRadialPos = initRadialPos;
		  goalVerticalPos = initVerticalPos;
		  goalArmRotation = (goalArmRotation + previousArmRotation) / 2.0f;
		  goalGripperRotation = initGripperRotation;
		  goalGripperPos = initGripperPos;
		}
	    } else // else if result has detection:
	    {
	      // update internal variables
	      foundSample = true;
	      sX = sStateImage.response.x;
	      sY = sStateImage.response.y;
	      // if the payload bay's current image-space position is within the allowable radius
	      if ( abs(sX) <= positionRadius && abs(sY) <= positionRadius ) 
		{
		  // update the internalSampleState
		  internalSampleState.pos.r = currentRadialPos;
		  internalSampleState.pos.theta = currentArmRotation;
		  internalSampleState.pos.z = currentVerticalPos;
		  internalSampleState.orientation.theta = sStateImage.response.angle;
		} else // need to center the payload
		{
		  // move to detected position (i.e. set goals to detected position)
		  // if sY > 0 : extend radius, else if sY < 0 retract radius
		  if ( sY > 0 )
		    {
		      initRadialPos += radialPosStep;
		    } else
		    {
		      initRadialPos -= radialPosStep;
		    }
		  // if sX > 0 : rotate CW, else if sX < 0 rotate CCW
		  if ( sX > 0 )
		    {
		      initArmRotation -= armRotationStep / 2.0f;
		    } else
		    {
		      initArmRotation += armRotationStep / 2.0f;
		    }
		}
	    }
	}
    } else // else sample has been found and is within radius
    {
      // set component's SampleState
      sample = internalSampleState;
      // transition to next state (GRABBING_SAMPLE)
      currentState = GRABBING_SAMPLE;
    }
}

void arm_controller::Grabbing_Sample_StateFunc()
{

  call_at_timer = false;

  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
  static float sampleZPlane = minVerticalPos;
  static bool atSample = false;
  static bool grabbedSample = false;

  // starting with:
  // * known sample position
  // * directly above sample

  // State logic:
  if ( !atSample )
    {
      // Orient gripper to sample (based on sample.orientation.theta)
      goalGripperRotation = sample.orientation.theta;
      // Go down to proper Z level for the sample (HOW DO WE DETERMINE THIS)
      goalVerticalPos = sampleZPlane;
      atSample = true;
    } else if ( !grabbedSample )
    {
      // close gripper
      goalGripperPos = gripperPosClosed;
      grabbedSample = true;
    } else
    {
      // move up some amount
      goalVerticalPos = maxVerticalPos;
      // transition to next state (CARRYING_SAMPLE)
      currentState = CARRYING_SAMPLE;
    }

}

void arm_controller::Carrying_Sample_StateFunc()
{

  call_at_timer = false;

  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
  static bool atPayloadBay = false;

  // starting with:
  // * already have the sample
  // * at max height
  // * already know the position & orientation of the PB

  // State Logic:
  if ( !atPayloadBay )
    {
      // move R,Theta to payloadBay's R,Theta
      goalArmRotation = payloadBay.pos.theta;
      goalRadialPos = payloadBay.pos.r;
      // change gripper rotation to payloadBay's orientation (payloadBay.orientation.theta)
      goalGripperRotation = payloadBay.orientation.theta;
    } else
    {
      // move down in Z to proper height for PB (HOW DO WE DETERMINE THIS? FROM MARKERS?)
      // transition to next state (INSERTING_SAMPLE)
      currentState = INSERTING_SAMPLE;
    }
}

void arm_controller::Inserting_Sample_StateFunc()
{

  call_at_timer = false;

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
  goalGripperPos = gripperPosOpened;
  // transition to next state (CLOSING_PB)
  currentState = CLOSING_PB;
}

void arm_controller::Closing_PB_StateFunc()
{

  call_at_timer = false;

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
  goalVerticalPos = maxVerticalPos;
  // send the command to the PB through UART to close the PB,
  // OPTIONAL : use image based detection to confirm PB closes
  // transition to next state (MOVING_AWAY) if PB responds well
  currentState = MOVING_AWAY;
}

void arm_controller::Moving_Away_StateFunc()
{

  call_at_timer = false;

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
  float safeRotation;
  safeRotation = payloadBay.pos.theta - 90.0f;
  if ( safeRotation < minArmRotation || safeRotation > maxArmRotation )
    safeRotation += 180.0f;
  // move to safe zone
  goalArmRotation = safeRotation;
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

  // need to initialize the min and max sensor values
  maxRadialPos       = 4000;
  maxVerticalPos     = 3000;
  maxArmRotation     = 330.0f;
  maxGripperRotation = 90.0f;
  maxGripperPos      = 180.0f;

  minRadialPos       = 300;
  minVerticalPos     = 100;
  minArmRotation     = 0.0f;
  minGripperRotation = 0.0f;
  minGripperPos      = 0.0f;

  // need to initialize the gripper's state sensor values
  gripperPosOpened = 0.0f;
  gripperPosClosed = 90.0f;

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
      if ( !CheckGoals() && !call_at_timer )
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
      sampleState_pub.publish(sample);
      payloadBayState_pub.publish(payloadBay);
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
    sampleState_pub.shutdown();
    payloadBayState_pub.shutdown();
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
    std::map<std::string,std::string> *portGroupMap = NULL;
    std::string configFileName = nodeName + "." + compName + ".xml";
    if (groupParser.Parse(configFileName))
    {
	portGroupMap = &groupParser.portGroupMap;
    }

    std::string advertiseName;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    advertiseName = "controlInputs";
    if ( portGroupMap != NULL && portGroupMap->find("controlInputs_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["controlInputs_sub"];
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&arm_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all publishers associated with this component
    // publisher: sampleState_pub
    advertiseName = "sampleState";
    if ( portGroupMap != NULL && portGroupMap->find("sampleState_pub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["sampleState_pub"];
    this->sampleState_pub = nh.advertise<agse_package::sampleState>
	(advertiseName.c_str(), 1000);	
    // publisher: payloadBayState_pub
    advertiseName = "payloadBayState";
    if ( portGroupMap != NULL && portGroupMap->find("payloadBayState_pub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["payloadBayState_pub"];
    this->payloadBayState_pub = nh.advertise<agse_package::payloadBayState>
	(advertiseName.c_str(), 1000);	

    // Configure all required services associated with this component
    // client: sampleStateFromImage_client
    advertiseName = "sampleStateFromImage";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->sampleStateFromImage_client = nh.serviceClient<agse_package::sampleStateFromImage>
	(advertiseName.c_str()); 
    // client: radialPos_client
    advertiseName = "radialPos";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->radialPos_client = nh.serviceClient<agse_package::radialPos>
	(advertiseName.c_str()); 
    // client: armRotation_client
    advertiseName = "armRotation";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->armRotation_client = nh.serviceClient<agse_package::armRotation>
	(advertiseName.c_str()); 
    // client: gripperRotation_client
    advertiseName = "gripperRotation";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->gripperRotation_client = nh.serviceClient<agse_package::gripperRotation>
	(advertiseName.c_str()); 
    // client: verticalPos_client
    advertiseName = "verticalPos";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->verticalPos_client = nh.serviceClient<agse_package::verticalPos>
	(advertiseName.c_str()); 
    // client: gripperPos_client
    advertiseName = "gripperPos";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->gripperPos_client = nh.serviceClient<agse_package::gripperPos>
	(advertiseName.c_str()); 
    // client: payloadBayStateFromImage_client
    advertiseName = "payloadBayStateFromImage";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->payloadBayStateFromImage_client = nh.serviceClient<agse_package::payloadBayStateFromImage>
	(advertiseName.c_str()); 

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


    /*
     * Identify present working directory of node executable
     */
    std::string s = node_argv[0];
    std::string exec_path = s;
    std::string delimiter = "/";
    std::string exec, pwd;
    size_t pos = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        exec = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
    }
    exec = s.substr(0, pos);
    pwd = exec_path.erase(exec_path.find(exec), exec.length());
    // Establish the log file name
    std::string log_file_path = pwd + nodeName + "." + compName + ".log"; 

    // Create the log file & open file stream
    LOGGER.CREATE_FILE(log_file_path);

    // Establish log levels of LOGGER
    LOGGER.SET_LOG_LEVELS(groupParser.logging);
}
