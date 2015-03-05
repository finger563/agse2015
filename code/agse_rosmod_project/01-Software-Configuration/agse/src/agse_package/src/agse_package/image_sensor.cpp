#include "agse_package/image_sensor.hpp"

//# Start User Globals Marker

uint8_t *buffer;

static int xioctl(int fd, int request, void *arg)
{
  int r;
  do r = v4l2_ioctl (fd, request, arg);
  while (-1 == r && ((EINTR == errno) || (errno == EAGAIN)));
  return r;
}

int print_caps(int fd, int width, int height)
{
  struct v4l2_format fmt = {0};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
        
  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
    {
      perror("Setting Pixel Format");
      return 1;
    }

  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24 ) {
    perror("couldn't set BGR24 format");
    return 1;
  }

  if (fmt.fmt.pix.width != width || fmt.fmt.pix.height != height ) {
    perror("couldn't set the height or width");
    return 1;
  }
 
  printf( "Selected Camera Mode:\n"
	  "  Width: %d\n"
	  "  Height: %d\n"
	  "  PixFmt: %d\n"
	  "  Field: %d\n",
	  fmt.fmt.pix.width,
	  fmt.fmt.pix.height,
	  fmt.fmt.pix.pixelformat,
	  fmt.fmt.pix.field);
  return 0;
}
 
int init_mmap(int fd)
{
  struct v4l2_requestbuffers req = {0};
  req.count = 1;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
 
  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
      perror("Requesting Buffer");
      return 1;
    }
 
  struct v4l2_buffer buf = {0};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
      perror("Querying Buffer");
      return 1;
    }
 
  buffer = (uint8_t *)v4l2_mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
  printf("Length: %d\nAddress: %p\n", buf.length, buffer);
  printf("Image Length: %d\n", buf.bytesused);
 
  return 0;
}

int tear_down(int fd)
{
  struct v4l2_buffer buf = {0};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
      perror("Query Buffer");
    }
  v4l2_munmap(buffer,buf.length);
  v4l2_close(fd);
}
 
int capture_image(int fd)
{
  struct v4l2_buffer buf = {0};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = 0;
  if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
      perror("Query Buffer");
      return 1;
    }
 
  if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
      perror("Start Capture");
      return 1;
    }
 
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd, &fds);
  struct timeval tv = {0};
  tv.tv_sec = 2;
  int r = select(fd+1, &fds, NULL, NULL, &tv);
  if(-1 == r)
    {
      perror("Waiting for Frame");
      return 1;
    }
 
  if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
      perror("Retrieving Frame");
      return 1;
    }
 
  return 0;
}
 
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
  width = 640;
  height = 480;

  videoFD = v4l2_open(videoDevice, O_RDWR);
  if (videoFD == -1)
    {
      perror("Opening video device");
    }
  else
    {
      if(print_caps(videoFD,width,height))
	ROS_INFO("Couldn't get or set camera capabilities with v4l.");
        
      if(init_mmap(videoFD))
	ROS_INFO("ERROR: couldn't init mmap!");
    }

  // Stop Init Timer
  initOneShotTimer.stop();
}
//# End Init Marker

// OnOneData Subscription handler for controlInputs_sub subscriber
//# Start controlInputs_sub_OnOneData Marker
void image_sensor::controlInputs_sub_OnOneData(const agse_package::controlInputs::ConstPtr& received_data)
{
  // Business Logic for controlInputs_sub subscriber subscribing to topic controlInputs callback 
  paused = received_data->paused;
  ROS_INFO( paused ? "Image Sensor paused!" : "Image Sensor Unpaused!" );
}
//# End controlInputs_sub_OnOneData Marker

// Component Service Callback
//# Start captureImageCallback Marker
bool image_sensor::captureImageCallback(agse_package::captureImage::Request  &req,
					agse_package::captureImage::Response &res)
{
  if (paused)
    {
      // Business Logic for captureImage_server Server providing captureImage Service
      struct v4l2_buffer buf = {0};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = 0;
      if(-1 == xioctl(videoFD, VIDIOC_QBUF, &buf))
	{
	  perror("Query Buffer");
	  return false;
	}
 
      if(-1 == xioctl(videoFD, VIDIOC_STREAMON, &buf.type))
	{
	  perror("Start Capture");
	  return false;
	}
 
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(videoFD, &fds);
      struct timeval tv = {0};
      tv.tv_sec = 2;
      int r = select(videoFD+1, &fds, NULL, NULL, &tv);
      if(-1 == r)
	{
	  perror("Waiting for Frame");
	  return false;
	}
 
      if(-1 == xioctl(videoFD, VIDIOC_DQBUF, &buf))
	{
	  perror("Retrieving Frame");
	  return false;
	}

      if(-1 == xioctl(videoFD, VIDIOC_STREAMOFF, &buf.type))
	{
	  perror("Stop Capture");
	  return false;
	}
    
      std::copy(buffer, buffer + buf.bytesused, back_inserter(res.imgVector));

      res.width = width;
      res.height = height;
		
      return true;
    }
  else
    {
      return false;
    }
}
//# End captureImageCallback Marker

// ---------------------------------------------
// EVERYTHING BELOW HERE IS COMPLETELY GENERATED
// ---------------------------------------------

// Destructor - required for clean shutdown when process is killed
image_sensor::~image_sensor()
{
    controlInputs_sub.shutdown();
    captureImage_server.shutdown();
//# Start Destructor Marker
    tear_down(videoFD);
//# End Destructor Marker
}

void image_sensor::startUp()
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
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName) != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName];
    ros::SubscribeOptions controlInputs_sub_options;
    controlInputs_sub_options = 
	ros::SubscribeOptions::create<agse_package::controlInputs>
	    (advertiseName.c_str(),
	     1000,
	     boost::bind(&image_sensor::controlInputs_sub_OnOneData, this, _1),
	     ros::VoidPtr(),
             &this->compQueue);
    this->controlInputs_sub = nh.subscribe(controlInputs_sub_options);

    // Configure all provided services associated with this component
    // server: captureImage_server
    advertiseName = "captureImage";
    if ( portGroupMap != NULL && portGroupMap->find(advertiseName) != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)[advertiseName];
    ros::AdvertiseServiceOptions captureImage_server_options;
    captureImage_server_options = 
	ros::AdvertiseServiceOptions::create<agse_package::captureImage>
	    (advertiseName.c_str(),
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
