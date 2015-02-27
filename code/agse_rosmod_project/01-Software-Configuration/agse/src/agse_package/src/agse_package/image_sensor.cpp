#include "agse_package/image_sensor.hpp"

//# Start User Globals Marker

//# End User Globals Marker

// -------------------------------------------------------
// BUSINESS LOGIC OF THESE FUNCTIONS SUPPLIED BY DEVELOPER
// -------------------------------------------------------

// Init Function
//# Start Init Marker
void image_sensor::Init(const ros::TimerEvent& event)
{
    // Initialize Component
  paused = true;
  sprintf(videoDevice,"/dev/video0");
  format = V4L2_PIX_FMT_MJPEG;
  width = 640;
  height = 480;
  brightness = contrast = saturation = gain = 0;
  quality = 95;
  grabMethod = 1;
  videoIn = (struct vdIn *) calloc (1, sizeof (struct vdIn));
  if (init_videoIn(videoIn, (char *) videoDevice, width, height, format, grabMethod) < 0)
    ROS_INFO("ERROR RUNNING INIT_VIDEOIN");
  v4l2ResetControl (videoIn, V4L2_CID_BRIGHTNESS);
  v4l2ResetControl (videoIn, V4L2_CID_CONTRAST);
  v4l2ResetControl (videoIn, V4L2_CID_SATURATION);
  v4l2ResetControl (videoIn, V4L2_CID_GAIN);

  //Setup Camera Parameters
  if (brightness != 0) {
    v4l2SetControl (videoIn, V4L2_CID_BRIGHTNESS, brightness);
  } 
  if (contrast != 0) {
    v4l2SetControl (videoIn, V4L2_CID_CONTRAST, contrast);
  } 
  if (saturation != 0) {
    v4l2SetControl (videoIn, V4L2_CID_SATURATION, saturation);
  } 
  if (gain != 0) {
    v4l2SetControl (videoIn, V4L2_CID_GAIN, gain);
  } 

    // Stop Init Timer
    initOneShotTimer.stop();
}
//# End Init Marker

// Component Service Callback
//# Start captureImageCallback Marker
bool image_sensor::captureImageCallback(agse_package::captureImage::Request  &req,
    agse_package::captureImage::Response &res)
{
    // Business Logic for captureImage_server Server providing captureImage Service
  if (uvcGrab (videoIn) < 0) 
    {
      ROS_INFO("ERROR GRABBING VIDEO");
    }
  res.imgVector.reserve(videoIn->buf.bytesused);
  //res.imgVector.insert(res.imgVector.end(), &videoIn->tmpbuffer[0], videoIn->buf.bytesused + DHT_SIZE);
  std::copy(&videoIn->tmpbuffer[0], &videoIn->tmpbuffer[0] + videoIn->buf.bytesused + DHT_SIZE, back_inserter(res.imgVector));
  return true;
}
//# End captureImageCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
image_sensor::~image_sensor()
{
    captureImage_server.shutdown();
}

void image_sensor::startUp()
{
    ros::NodeHandle nh;

    // Configure all provided services associated with this component
    // server: captureImage_server
    ros::AdvertiseServiceOptions captureImage_server_options;
    captureImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::captureImage>
	    ("captureImage",
             boost::bind(&image_sensor::captureImageCallback, this, _1, _2),
	     ros::VoidPtr(),
             &this->compQueue);
    this->captureImage_server = nh.advertiseService(captureImage_server_options);
 
    // Create Init Timer
    ros::TimerOptions timer_options;
    timer_options = 
	ros::TimerOptions
	    (ros::Duration(-1),
	     boost::bind(&image_sensor::Init, this, _1),
	     &this->compQueue,
             true);
    this->initOneShotTimer = nh.createTimer(timer_options);  
  
}
