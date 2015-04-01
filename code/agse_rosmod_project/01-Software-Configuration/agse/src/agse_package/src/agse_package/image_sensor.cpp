#include "agse_package/image_sensor.hpp"

//# Start User Globals Marker
struct buffer      *buffers;
struct v4l2_buffer  buf;
int                 n_buffers;

static void xioctl(int fh, int request, void *arg)
{
  int r;
  do {
    r = v4l2_ioctl(fh, request, arg);
  } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));
  if (r == -1) {
    fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
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
  width = 1920;
  height = 1080;

  struct v4l2_format              fmt;
  struct v4l2_requestbuffers      reqBufs;

  videoFD = v4l2_open(videoDevice, O_RDWR | O_NONBLOCK, 0);
  if (videoFD < 0) {
    perror("Cannot open device");
    exit(EXIT_FAILURE);
  }

  CLEAR(fmt);
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = width;
  fmt.fmt.pix.height      = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
  fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
  xioctl(videoFD, VIDIOC_S_FMT, &fmt);
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24) {
    printf("Libv4l didn't accept BGR24 format. Can't proceed.\n");
    exit(EXIT_FAILURE);
  }
  if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height))
    printf("Warning: driver is sending image at %dx%d\n",
	   fmt.fmt.pix.width, fmt.fmt.pix.height);

  CLEAR(reqBufs);
  reqBufs.count = 2;
  reqBufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqBufs.memory = V4L2_MEMORY_MMAP;
  xioctl(videoFD, VIDIOC_REQBUFS, &reqBufs);

  buffers = (buffer *)calloc(reqBufs.count, sizeof(*buffers));
  for (n_buffers = 0; n_buffers < reqBufs.count; ++n_buffers) {
    CLEAR(buf);

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    xioctl(videoFD, VIDIOC_QUERYBUF, &buf);

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
					 PROT_READ | PROT_WRITE, MAP_SHARED,
					 videoFD, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start) {
      perror("mmap");
      exit(EXIT_FAILURE);
    }
  }

  // Command line args for servo control
  for (int i = 0; i < node_argc; i++) 
    {
      if (!strcmp(node_argv[i], "-unpaused"))
	paused = false;
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
  if (!paused)
    {
      // Business Logic for captureImage_server Server providing captureImage Service
      fd_set                          fds;
      struct timeval                  tv;
      enum v4l2_buf_type              type;
      int                             r;

      for (int i = 0; i < n_buffers; ++i) {
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;
	xioctl(videoFD, VIDIOC_QBUF, &buf);
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      xioctl(videoFD, VIDIOC_STREAMON, &type);
      do {
	FD_ZERO(&fds);
	FD_SET(videoFD, &fds);

	/* Timeout. */
	tv.tv_sec = 2;
	tv.tv_usec = 0;

	r = select(videoFD + 1, &fds, NULL, NULL, &tv);
      } while ((r == -1 && (errno = EINTR)));
      if (r == -1) {
	perror("select");
	return false;
      }

      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      xioctl(videoFD, VIDIOC_DQBUF, &buf);

      //fwrite(buffers[buf.index].start, buf.bytesused, 1, fout);

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(videoFD, VIDIOC_STREAMOFF, &type);

      res.imgVector.reserve(buf.bytesused);
      std::copy(&((unsigned char *)buffers[buf.index].start)[0],
		&((unsigned char *)buffers[buf.index].start)[0] + buf.bytesused,
		back_inserter(res.imgVector));
      res.width = width;
      res.height = height;
		
      return true;
    }
  return false;
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
  for (int i = 0; i < n_buffers; ++i)
    v4l2_munmap(buffers[i].start, buffers[i].length);
  v4l2_close(videoFD);
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
    if ( portGroupMap != NULL && portGroupMap->find("controlInputs_sub") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["controlInputs_sub"];
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
    if ( portGroupMap != NULL && portGroupMap->find("captureImage_server") != portGroupMap->end() )
        advertiseName += "_" + (*portGroupMap)["captureImage_server"];
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
