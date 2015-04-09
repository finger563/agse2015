#include "agse_package/user_input_imager.hpp"

//# Start User Globals Marker
using namespace cv;
#include "agse_package/uip.h"
//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void user_input_imager::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  ROS_INFO("Initializing User Input Imager");

  //////////////////////////////////////////////
  // UIP LCD SETUP
  //////////////////////////////////////////////
  
  // The Four Images to show in UIP
  //  top_right = cvLoadImage("/home/debian/Repositories/agse2015/code/UIP/input/white.png");
  //  bottom_right = cvLoadImage("/home/debian/Repositories/agse2015/code/UIP/input/white.png");

  key = 0;

  Mode_1 = cvCreateImage( cvSize(800, 480), 8, 3);
  Mode_2 = cvCreateImage( cvSize(800, 480), 8, 3);
  Mode_3 = cvCreateImage( cvSize(800, 480), 8, 3);
  Mode_4 = cvCreateImage( cvSize(800, 480), 8, 3);
  processed_image = cvCreateImage(cvSize(1920, 1080), 8, 3);
    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for payloadBayDetectionImages_sub subscriber
//# Start payloadBayDetectionImages_sub_OnOneData Marker
void user_input_imager::payloadBayDetectionImages_sub_OnOneData(const agse_package::payloadBayDetectionImages::ConstPtr& received_data)
{

  ROS_INFO("Payload Bay Subscriber::Received processed payload bay images");

    // Business Logic for payloadBayDetectionImages_sub subscriber subscribing to topic payloadBayDetectionImages callback
  pb_rawImage = Mat(received_data->height, 
		    received_data->width, 
		    CV_8UC3, 
		    const_cast<unsigned char*>(received_data->rawImgVector.data()));

  pb_hsvImage = Mat(received_data->height, 
		    received_data->width, 
		    CV_8UC3, 
		    const_cast<unsigned char*>(received_data->hsvThreshImgVector.data()));

  pb_gsImage = Mat(received_data->height, 
		   received_data->width, 
		   CV_8UC3, 
		   const_cast<unsigned char*>(received_data->gsThreshImgVector.data()));
  
  pb_bitwise = Mat(received_data->height, 
		   received_data->width, 
		   CV_8UC3, 
		   const_cast<unsigned char*>(received_data->bitwiseAndImgVector.data()));

  bottom_left = cvCreateImage(cvSize(pb_rawImage.cols, pb_rawImage.rows), 8, 3);
  IplImage ipltemp = pb_rawImage;
  cvCopy(&ipltemp, bottom_left);

}
//# End payloadBayDetectionImages_sub_OnOneData Marker
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
// OnOneData Subscription handler for sampleDetectionImages_sub subscriber
//# Start sampleDetectionImages_sub_OnOneData Marker
void user_input_imager::sampleDetectionImages_sub_OnOneData(const agse_package::sampleDetectionImages::ConstPtr& received_data)
{

  ROS_INFO("Sample Subscriber::Received processed sample images");

    // Business Logic for sampleDetectionImages_sub subscriber subscribing to topic sampleDetectionImages callback 
  sample_rawImage = Mat(received_data->height, 
			received_data->width, 
			CV_8UC3, 
			const_cast<unsigned char*>(received_data->rawImgVector.data()));

  sample_hsvImage = Mat(received_data->height, 
			received_data->width, 
			CV_8UC3, 
			const_cast<unsigned char*>(received_data->hsvThreshImgVector.data()));

  sample_gsImage = Mat(received_data->height, 
		       received_data->width, 
		       CV_8UC3, 
		       const_cast<unsigned char*>(received_data->gsThreshImgVector.data()));
  
  sample_bitwise = Mat(received_data->height, 
		       received_data->width, 
		       CV_8UC3, 
		       const_cast<unsigned char*>(received_data->bitwiseAndImgVector.data()));

  top_left = cvCreateImage(cvSize(sample_rawImage.cols, sample_rawImage.rows), 8, 3);
  IplImage ipltemp = sample_rawImage;
  cvCopy(&ipltemp, top_left);

}
//# End sampleDetectionImages_sub_OnOneData Marker

// Callback for uiImage_timer timer
//# Start uiImage_timerCallback Marker
void user_input_imager::uiImage_timerCallback(const ros::TimerEvent& event)
{
  ROS_INFO("UIP Imager Timer Callback");
    // Business Logic for uiImage_timer 
  agse_package::captureImage arg;
  Mat camera_feed;

  if (this->captureImage_client.call(arg)) {

    ROS_INFO("Capture Image Client Call Successful; Height: %d, Width: %d ", arg.response.height, arg.response.width);
    
    camera_feed = Mat(arg.response.height, 
		      arg.response.width, 
		      CV_8UC3, 
		      arg.response.imgVector.data());

    // Mat to IplImage *
    IplImage ipltemp = camera_feed;
    cvCopy(&ipltemp, processed_image);
    cvResize(processed_image, Mode_1);
    cvShowImage( "UIP", Mode_1);
    cvNamedWindow( "UIP", 1 );
    cvSetWindowProperty("UIP", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
  }

  key = 0;
  key = cvWaitKey(1);

  // RAW CAMERA FEED
  if (key == 65361) {
    ROS_INFO("Mode 1 Activated");
    cvShowImage( "UIP", Mode_1);
  }
  /*
  // SAMPLE PROCESSED IMAGE
  else if (key == 65363) {
    ROS_INFO("Mode 2 Activated");

    // Mat to IplImage *
    processed_image = cvCreateImage(cvSize(sample_rawImage.cols, sample_rawImage.rows), 8, 3);
    IplImage ipltemp = sample_rawImage;
    cvCopy(&ipltemp, processed_image);

    cvResize(processed_image, Mode_2);
    cvShowImage( "UIP", Mode_2);
    cvNamedWindow( "UIP", 1 );
    cvSetWindowProperty("UIP", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    key = 0;

  }

  else if (key == 65362) {

    ROS_INFO("Mode 3 Activated");

    // Mat to IplImage *
    processed_image = cvCreateImage(cvSize(pb_rawImage.cols, pb_rawImage.rows), 8, 3);
    IplImage ipltemp = pb_rawImage;
    cvCopy(&ipltemp, processed_image);

    cvResize(processed_image, Mode_3);
    cvShowImage( "UIP", Mode_3);
    cvNamedWindow( "UIP", 1 );
    cvSetWindowProperty("UIP", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    key = 0;
  }

  else if (key == 65364) {

    ROS_INFO("Mode 4 Activated"); 

    // Mat to IplImage *
    processed_image = cvCreateImage(cvSize(pb_gsImage.cols, pb_gsImage.rows), 8, 3);
    IplImage ipltemp = pb_gsImage;
    cvCopy(&ipltemp, processed_image);

    cvResize(processed_image, Mode_4);
    cvShowImage( "UIP", Mode_4);
    cvNamedWindow( "UIP", 1 );
    cvSetWindowProperty("UIP", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    key = 0;
  }

  else if (key == 13) {
      
    ROS_INFO("Mode 5 Activated");
    cvShowManyImages("UIP", 4, top_left, top_right, bottom_left, bottom_right);
    key = 0;
  }
  */
  else {
    ROS_INFO("No Mode Activated");
    cvShowImage( "UIP", Mode_1);
  }

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
