#include "agse_package/servo_controller.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void servo_controller::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;
  sprintf(portName,"//dev//ttyO5");
  armServoID = 1;
  gripperRotationID = 10;
  gripperPositionID = 11;
  if (serialPort.connect(portName)!=0)
    {
      armRotationGoal = 0;
      gripperRotationGoal = 0;
      gripperPosGoal = 0;
    }
  else
    {
      ROS_INFO ("Can't open serial port %s", portName);
    }  
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void servo_controller::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
    // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Servos paused!" : "Servos Unpaused" );
}
//# End controlInputs_sub_OnOneData Marker

// Component Service Callback
//# Start armRotationCallback Marker
bool servo_controller::armRotationCallback(agse_package::armRotation::Request  &req,
    agse_package::armRotation::Response &res)
{
    // Business Logic for armRotation_server Server providing armRotation Service
  if (req.update == true)
    {
      armRotationGoal = req.goal;
    }
  res.current = armRotationCurrent;
  return true;
}
//# End armRotationCallback Marker
// Component Service Callback
//# Start gripperPosCallback Marker
bool servo_controller::gripperPosCallback(agse_package::gripperPos::Request  &req,
    agse_package::gripperPos::Response &res)
{
    // Business Logic for gripperPos_server Server providing gripperPos Service
  if (req.update == true)
    {
      gripperPosGoal = req.goal;
    }
  res.current = gripperPosCurrent;
  return true;
}
//# End gripperPosCallback Marker
// Component Service Callback
//# Start gripperRotationCallback Marker
bool servo_controller::gripperRotationCallback(agse_package::gripperRotation::Request  &req,
    agse_package::gripperRotation::Response &res)
{
    // Business Logic for gripperRotation_server Server providing gripperRotation Service
  if (req.update == true)
    {
      gripperRotationGoal = req.goal;
    }
  res.current = gripperRotationCurrent;
  return true;
}
//# End gripperRotationCallback Marker

// Callback for servoTimer timer
//# Start servoTimerCallback Marker
void servo_controller::servoTimerCallback(const ros::TimerEvent& event)
{
    // Business Logic for servoTimer 
  if (paused) {

    armRotationGoal = Dynamixel::posToAngle( rand() % 1023 );
    gripperRotationGoal = Dynamixel::posToAngle( rand() % 1023 );
    gripperPosGoal = Dynamixel::posToAngle( rand() % 1023 );

    int pos; // temp value to store position from servo
    
    // ARM SERVO 
    dynamixel.setPosition(&serialPort, armServoID, Dynamixel::angleToPos(armRotationGoal));
    pos = dynamixel.getPosition(&serialPort, armServoID);
    ROS_INFO("Arm base servo angle: %f\n",Dynamixel::posToAngle(pos));
    armRotationCurrent = Dynamixel::posToAngle(pos);

    // GRIPPER ROTATION SERVO
    dynamixel.setPosition(&serialPort, gripperRotationID, Dynamixel::angleToPos(gripperRotationGoal));
    pos = dynamixel.getPosition(&serialPort, gripperRotationID);
    ROS_INFO("Gripper rotation servo angle: %f\n",Dynamixel::posToAngle(pos));
    gripperRotationCurrent = Dynamixel::posToAngle(pos);
    
    // GRIPPER POSITION SERVO
    dynamixel.setPosition(&serialPort, gripperPositionID, Dynamixel::angleToPos(gripperPosGoal));
    pos = dynamixel.getPosition(&serialPort, gripperPositionID);
    ROS_INFO("Gripper position servo angle: %f\n",Dynamixel::posToAngle(pos));
    gripperPosCurrent = Dynamixel::posToAngle(pos);
  }
}
//# End servoTimerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
servo_controller::~servo_controller()
{
    servoTimer.stop();
    controlInputs_sub.shutdown();
    armRotation_server.shutdown();
    gripperPos_server.shutdown();
    gripperRotation_server.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void servo_controller::startUp()
{
    ros::NodeHandle nh;

    // Configure all subscribers associated with this component
    // subscriber: controlInputs_sub
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    ("controlInputs",
	     1000,
	     boost::bind(&servo_controller::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: armRotation_server
    ros::AdvertiseServiceOptions armRotation_server_options;
    armRotation_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::armRotation>
	    ("armRotation",
             boost::bind(&servo_controller::armRotationCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->armRotation_server = nh.advertiseService(armRotation_server_options);
    // server: gripperPos_server
    ros::AdvertiseServiceOptions gripperPos_server_options;
    gripperPos_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::gripperPos>
	    ("gripperPos",
             boost::bind(&servo_controller::gripperPosCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->gripperPos_server = nh.advertiseService(gripperPos_server_options);
    // server: gripperRotation_server
    ros::AdvertiseServiceOptions gripperRotation_server_options;
    gripperRotation_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::gripperRotation>
	    ("gripperRotation",
             boost::bind(&servo_controller::gripperRotationCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->gripperRotation_server = nh.advertiseService(gripperRotation_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&servo_controller::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(1.0),
	     boost::bind(&servo_controller::servoTimerCallback, this, _1),
	     &this->compQueue);
    this->servoTimer = nh.createTimer(timer_options);

}
