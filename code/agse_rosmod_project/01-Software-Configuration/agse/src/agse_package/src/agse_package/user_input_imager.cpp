#include "agse_package/user_input_imager.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void user_input_imager::Init(const ros::TimerEvent& event)
{
    // Initialize Component

    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for payloadBayDetectionImages_sub subscriber
//# Start payloadBayDetectionImages_sub_OnOneData Marker
void user_input_imager::payloadBayDetectionImages_sub_OnOneData(const agse_package::payloadBayDetectionImages::ConstPtr& received_data)
{
    // Business Logic for payloadBayDetectionImages_sub subscriber subscribing to topic payloadBayDetectionImages callback 

}
//# End payloadBayDetectionImages_sub_OnOneData Marker
// OnOneData Subscription handler for sampleDetectionImages_sub subscriber
//# Start sampleDetectionImages_sub_OnOneData Marker
void user_input_imager::sampleDetectionImages_sub_OnOneData(const agse_package::sampleDetectionImages::ConstPtr& received_data)
{
    // Business Logic for sampleDetectionImages_sub subscriber subscribing to topic sampleDetectionImages callback 

}
//# End sampleDetectionImages_sub_OnOneData Marker
// OnOneData Subscription handler for payloadBayState_sub subscriber
//# Start payloadBayState_sub_OnOneData Marker
void user_input_imager::payloadBayState_sub_OnOneData(const agse_package::payloadBayState::ConstPtr& received_data)
{
    // Business Logic for payloadBayState_sub subscriber subscribing to topic payloadBayState callback 

}
//# End payloadBayState_sub_OnOneData Marker
// OnOneData Subscription handler for sampleState_sub subscriber
//# Start sampleState_sub_OnOneData Marker
void user_input_imager::sampleState_sub_OnOneData(const agse_package::sampleState::ConstPtr& received_data)
{
    // Business Logic for sampleState_sub subscriber subscribing to topic sampleState callback 

}
//# End sampleState_sub_OnOneData Marker

// Callback for uiImage_timer timer
//# Start uiImage_timerCallback Marker
void user_input_imager::uiImage_timerCallback(const ros::TimerEvent& event)
{
    // Business Logic for uiImage_timer 

}
//# End uiImage_timerCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
user_input_imager::~user_input_imager()
{
    uiImage_timer.stop();
    payloadBayDetectionImages_sub.shutdown();
    sampleDetectionImages_sub.shutdown();
    payloadBayState_sub.shutdown();
    sampleState_sub.shutdown();
    captureImage_client.shutdown();
//# Start Destructor Marker

//# End Destructor Marker
}

void user_input_imager::startUp()
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
    // subscriber: payloadBayDetectionImages_sub
    advertiseName = "payloadBayDetectionImages";
    if ( portGroupMap != NULL && portGroupMap->find("payloadBayDetectionImages_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["payloadBayDetectionImages_sub"];
    ros::SubscribeOptions payloadBayDetectionImages_sub_options;
    payloadBayDetectionImages_sub_options = 
	ros::SubscribeOptions::create<agse_package::payloadBayDetectionImages>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_imager::payloadBayDetectionImages_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->payloadBayDetectionImages_sub = nh.subscribe(payloadBayDetectionImages_sub_options);
    // subscriber: sampleDetectionImages_sub
    advertiseName = "sampleDetectionImages";
    if ( portGroupMap != NULL && portGroupMap->find("sampleDetectionImages_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["sampleDetectionImages_sub"];
    ros::SubscribeOptions sampleDetectionImages_sub_options;
    sampleDetectionImages_sub_options = 
	ros::SubscribeOptions::create<agse_package::sampleDetectionImages>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_imager::sampleDetectionImages_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->sampleDetectionImages_sub = nh.subscribe(sampleDetectionImages_sub_options);
    // subscriber: payloadBayState_sub
    advertiseName = "payloadBayState";
    if ( portGroupMap != NULL && portGroupMap->find("payloadBayState_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["payloadBayState_sub"];
    ros::SubscribeOptions payloadBayState_sub_options;
    payloadBayState_sub_options = 
	ros::SubscribeOptions::create<agse_package::payloadBayState>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_imager::payloadBayState_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->payloadBayState_sub = nh.subscribe(payloadBayState_sub_options);
    // subscriber: sampleState_sub
    advertiseName = "sampleState";
    if ( portGroupMap != NULL && portGroupMap->find("sampleState_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["sampleState_sub"];
    ros::SubscribeOptions sampleState_sub_options;
    sampleState_sub_options = 
	ros::SubscribeOptions::create<agse_package::sampleState>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&user_input_imager::sampleState_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->sampleState_sub = nh.subscribe(sampleState_sub_options);

    // Configure all required services associated with this component
    // client: captureImage_client
    advertiseName = "captureImage";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName+"_client") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName+"_client"];
    this->captureImage_client = nh.serviceClient<agse_package::captureImage>
	(advertiseName.c_str()); 

    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&user_input_imager::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
    // Create all component timers
    // timer: timer.properties["name"]
    timer_options = 
	ros::TimerOptions
             (ros::Duration(0.5),
	     boost::bind(&user_input_imager::uiImage_timerCallback, this, _1),
	     &this->compQueue);
    this->uiImage_timer = nh.createTimer(timer_options);


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
