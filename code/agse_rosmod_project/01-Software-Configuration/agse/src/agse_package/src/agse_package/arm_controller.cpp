#include "agse_package/arm_controller.hpp"

//# Start User Globals Marker

void arm_controller::UpdateSensorData()
{
  // all servers have 
  // inputs: int64 goal, bool update
  // output: current
  agse_package::radialPos rPos;
  rPos.request.update = false;
  rPos.request.setZeroPosition = false;
  radialPos_client.call(rPos);
  currentRadialPos = rPos.response.current;

  agse_package::verticalPos vPos;
  vPos.request.update = false;
  vPos.request.setZeroPosition = false;
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
  rPos.request.setZeroPosition = false;
  rPos.request.goal = goalRadialPos;
  radialPos_client.call(rPos);
  currentRadialPos = rPos.response.current;

  agse_package::verticalPos vPos;
  vPos.request.update = true;
  vPos.request.setZeroPosition = false;
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
#if AGSE_DEBUG
  ROS_INFO("RADIAL GOAL: %d",goalRadialPos);
  ROS_INFO("RADIUS : %d",currentRadialPos);
  ROS_INFO("HEIGHT GOAL: %d",goalVerticalPos);
  ROS_INFO("HEIGHT : %d",currentVerticalPos);
  ROS_INFO("ARM ROTATION GOAL: %f",goalArmRotation);
  ROS_INFO("ARM ROTATION : %f",currentArmRotation);
  ROS_INFO("GRIPPER ROTATION GOAL: %f",goalGripperRotation);
  ROS_INFO("GRIPPER ROTATION : %f",currentGripperRotation);
  ROS_INFO("GRIPPER POSITION GOAL: %f",goalGripperPos);
  ROS_INFO("GRIPPER POSITION : %f",currentGripperPos);
#endif
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
  ROS_INFO("REACHED GOALS");
  return true;
}

void arm_controller::Init_StateFunc()
{
  // Init zeroes out the positions of the linear actuators for calibration
  static bool zeroedHeight = false;
  static bool zeroedRadius = false;
  if (!zeroedHeight)
    {
      ROS_INFO("ZEROING HEIGHT");
      agse_package::verticalPos vPos;
      vPos.request.update = true;
      vPos.request.setZeroPosition = false;
      vPos.request.goal = currentVerticalPos - 10000;
      verticalPos_client.call(vPos);
      if (vPos.response.lowerLimitReached)
	{
	  ROS_INFO("LOWER LIMIT REACHED: HEIGHT");
	  vPos.request.update = false;
	  vPos.request.setZeroPosition = true;
	  verticalPos_client.call(vPos);
	  zeroedHeight = true;
	}
    }
  else if (!zeroedRadius)
    {
      ROS_INFO("ZEROING RADIUS");
      agse_package::radialPos rPos;
      rPos.request.update = true;
      rPos.request.setZeroPosition = false;
      rPos.request.goal = currentRadialPos - 10000;
      radialPos_client.call(rPos);
      if (rPos.response.lowerLimitReached)
	{
	  ROS_INFO("LOWER LIMIT REACHED: RADIUS");
	  rPos.request.update = false;
	  rPos.request.setZeroPosition = true;
	  radialPos_client.call(rPos);
	  zeroedRadius = true;
	}
    }
  else
    {
      ROS_INFO("ZEROED EVERYTHING");
      agse_package::verticalPos vPos;
      vPos.request.update = true;
      vPos.request.setZeroPosition = false;
      vPos.request.goal = 0;
      verticalPos_client.call(vPos);
      agse_package::radialPos rPos;
      rPos.request.update = true;
      rPos.request.setZeroPosition = false;
      rPos.request.goal = 0;
      radialPos_client.call(rPos);

      goalRadialPos = minRadialPos;
      goalVerticalPos = minVerticalPos;
      goalArmRotation = minArmRotation;
      goalGripperRotation = currentGripperRotation;
      goalGripperPos = gripperPosOpened;

      currentState = FINDING_PB;
    }
}

void arm_controller::Finding_PB_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  static float initRadialPos       = (maxRadialPos + minRadialPos) / 2.0f;
  static float initVerticalPos     = minVerticalPos;
  static float initArmRotation     = minArmRotation;
  static float initGripperRotation = (maxGripperRotation + minGripperRotation) / 2.0f;
  static float initGripperPos      = gripperPosOpened;

  static float maxSearchTime = 300.0f; // seconds we are allowed to search

  static float armRotationStep = 30.0f; // degrees between steps of the state search
  static float radialPosStep = 1000.0f;    // amount to move by in radius

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
	  ROS_INFO("GOT PAYLOAD BAY STATE: %d, %f , %f, %f",pbStateImage.response.status, pbStateImage.response.x, pbStateImage.response.y, pbStateImage.response.angle);
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
		  // if pbY > 0 : rotate CW, else if pbY < 0 rotate CCW
		  if ( pbY > 0 )
		    {
		      initArmRotation += armRotationStep / 6.0f;
		    } else
		    {
		      initArmRotation -= armRotationStep / 6.0f;
		    }
		  // if pbX > 0 : retract radius, else if pbX < 0 extend radius
		  if ( pbX > 0 )
		    {
		      initRadialPos -= radialPosStep;
		    } else
		    {
		      initRadialPos += radialPosStep;
		    }
		  goalRadialPos = initRadialPos;
		  goalArmRotation = initArmRotation;
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
  static float initRadialPos       = (maxRadialPos + minRadialPos) / 2.0f;
  static float initVerticalPos     = minVerticalPos;
  static float initArmRotation     = minArmRotation;
  static float initGripperRotation = (maxGripperRotation + minGripperRotation) / 2.0f;
  static float initGripperPos      = gripperPosOpened;

  static float maxSearchTime = 300.0f; // seconds we are allowed to search

  static float armRotationStep = 30.0f; // degrees between steps of the state search
  static float radialPosStep = 1000.0f;    // amount to move by in radius

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
	  ROS_INFO("GOT SAMPLE STATE: %d, %f , %f, %f",sStateImage.response.status, sStateImage.response.x, sStateImage.response.y, sStateImage.response.angle);
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
		  // if sY > 0 : rotate CW, else if sY < 0 CCW
		  if ( sY > 0 )
		    {
		      initArmRotation += armRotationStep / 6.0f;
		    } else
		    {
		      initArmRotation -= armRotationStep / 6.0f;
		    }
		  // if sX > 0 : retract radius, else if sX < 0 extend
		  if ( sX > 0 )
		    {
		      initRadialPos -= radialPosStep;
		    } else
		    {
		      initRadialPos += radialPosStep;
		    }
		  goalRadialPos = initRadialPos;
		  goalArmRotation = initArmRotation;
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
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
  static float sampleZPlane = maxVerticalPos;
  static bool atSample = false;
  static bool grabbedSample = false;

  // starting with:
  // * known sample position
  // * directly above sample

  // State logic:
  if ( !atSample )
    {
      // Orient gripper to sample (based on sample.orientation.theta)
      goalGripperRotation = sample.orientation.theta + gripperRotationOffset;
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
      goalVerticalPos = minVerticalPos;
      // transition to next state (CARRYING_SAMPLE)
      currentState = CARRYING_SAMPLE;
    }
}

void arm_controller::Carrying_Sample_StateFunc()
{
  // initialize static members for initial values of this state
  //   e.g. where the search starts, what the goals of the state are, etc.
  // perform any image processing required using the detector
  // update the arm's goal variables based on the result of the image processing
  // update the current state of the arm if necessary
  static bool atPayloadBay = false;
  static float payloadBayZPlane = (maxVerticalPos+minVerticalPos)/4;

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
      goalGripperRotation = payloadBay.orientation.theta + gripperRotationOffset;
    } else
    {
      // move down in Z to proper height for PB (HOW DO WE DETERMINE THIS? FROM MARKERS?)
      goalVerticalPos = payloadBayZPlane;
      // transition to next state (INSERTING_SAMPLE)
      currentState = INSERTING_SAMPLE;
    }
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
  goalGripperPos = gripperPosOpened;
  // transition to next state (CLOSING_PB)
  currentState = CLOSING_PB;
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
  goalVerticalPos = minVerticalPos;
  // send the command to the PB through UART to close the PB,
  // OPTIONAL : use image based detection to confirm PB closes
  // transition to next state (MOVING_AWAY) if PB responds well
  currentState = MOVING_AWAY;
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

  // need to initialize the offsets with measurements from the system
  radialOffset          = 0.0;
  verticalOffset        = 0.0;
  armRotationOffset     = 0.0f;
  gripperRotationOffset = 0.0f;
  gripperPosOffset      = 0.0f;

  // need to initialize the offsets with measurements from the system
  radiusBetweenGripperAndCamera = -500;
  angleBetweenGripperAndCamera = 5.0f;

  // need to initialize the min and max sensor values
  maxRadialPos       = 275000;
  maxVerticalPos     = 495000;
  maxArmRotation     = 330.0f;
  maxGripperRotation = 180.0f;
  maxGripperPos      = 260.0f;

  minRadialPos       = 0;
  minVerticalPos     = 0;
  minArmRotation     = 0.0f;
  minGripperRotation = 0.0f;
  minGripperPos      = 190.0f;

  // need to initialize the gripper's state sensor values
  gripperPosOpened = 250.0f;
  gripperPosClosed = 200.0f;

  // need to properly initialize the current sensor readings
  currentRadialPos        = -1.0;
  currentVerticalPos      = -1.0;
  currentArmRotation      = -1.0f;
  currentGripperRotation  = -1.0f;
  currentGripperPos       = -1.0f;

  // need to properly initialize the current actuator goals
  goalRadialPos        = minRadialPos;
  goalVerticalPos      = minVerticalPos;
  goalArmRotation      = minArmRotation;
  goalGripperRotation  = minGripperRotation;
  goalGripperPos       = gripperPosOpened;

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
  radialEpsilon          = 100;
  verticalEpsilon        = 100;
  armRotationEpsilon     = 1.0f;
  gripperRotationEpsilon = 1.0f;
  gripperPosEpsilon      = 1.0f;

  // command line args parsing for arm_controller:
  for (int i=0; i < node_argc; i++) 
    {
      if (!strcmp(node_argv[i], "-unpaused"))
	{
	  paused = false;
	}
      if (!strcmp(node_argv[i], "-r"))
	{
	  goalRadialPos = atoi(node_argv[i+1]);
	}
      if (!strcmp(node_argv[i], "-theta"))
	{
	  goalArmRotation = atoi(node_argv[i+1]);
	}
      if (!strcmp(node_argv[i], "-z"))
	{
	  goalVerticalPos = atoi(node_argv[i+1]);
	}
      if (!strcmp(node_argv[i], "-gRot"))
	{
	  goalGripperRotation = atoi(node_argv[i+1]);
	}
      if (!strcmp(node_argv[i], "-gPos"))
	{
	  goalGripperPos = atoi(node_argv[i+1]);
	}
    }

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
	  ROS_INFO("FINDING PAYLOAD BAY");
	  if ( CheckGoals() )
	    Finding_PB_StateFunc();
	  UpdateArmPosition();
	  break;
	case OPENING_PB:
	  ROS_INFO("OPENING PAYLOAD BAY");
	  if ( CheckGoals() )
	    Opening_PB_StateFunc();
	  UpdateArmPosition();
	  break;
	case FINDING_SAMPLE:
	  ROS_INFO("FINDING SAMPLE");
	  if ( CheckGoals() )
	    Finding_Sample_StateFunc();
	  UpdateArmPosition();
	  break;
	case GRABBING_SAMPLE:
	  ROS_INFO("GRABBING SAMPLE");
	  if ( CheckGoals() )
	    Grabbing_Sample_StateFunc();
	  UpdateArmPosition();
	  break;
	case CARRYING_SAMPLE:
	  ROS_INFO("CARRYING SAMPLE");
	  if ( CheckGoals() )
	    Carrying_Sample_StateFunc();
	  UpdateArmPosition();
	  break;
	case INSERTING_SAMPLE:
	  ROS_INFO("INSERTING SAMPLE");
	  if ( CheckGoals() )
	    Inserting_Sample_StateFunc();
	  UpdateArmPosition();
	  break;
	case CLOSING_PB:
	  ROS_INFO("CLOSING PAYLOAD BAY");
	  if ( CheckGoals() )
	    Closing_PB_StateFunc();
	  UpdateArmPosition();
	  break;
	case MOVING_AWAY:
	  ROS_INFO("MOVING AWAY");
	  if ( CheckGoals() )
	    Moving_Away_StateFunc();
	  UpdateArmPosition();
	  break;
	default:
	  break;
	}
      sampleState_pub.publish(sample);
      payloadBayState_pub.publish(payloadBay);
      arm.state = currentState;
      armState_pub.publish(arm);
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
    armState_pub.shutdown();
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
    // publisher: armState_pub
    advertiseName = "armState";
    if ( portGroupMap != NULL && portGroupMap->find("armState_pub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["armState_pub"];
    this->armState_pub = nh.advertise<agse_package::armState>
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
             (ros::Duration(0.2),
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
