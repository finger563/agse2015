#ifndef PAYLOADBAY_DETECTOR_HPP
#define PAYLOADBAY_DETECTOR_HPP

#include <iostream>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <opencv2/highgui/highgui.hpp>

#include "agse_package/payloadBayStateFromImage.h"

using namespace cv;
using namespace aruco;

class PayloadBay_Detector {
public:
  void init(float msize, const char* camParamFileName);
  void run( std::vector<unsigned char> & raw_image_data, 
			   int width, 
			   int height, 
			   const char* fname = "");
private:
  MarkerDetector MDetector;
  // real-world size of the markers
  float MarkerSize;
  // the read-in yml file which describes camera intrinsics
  aruco::CameraParameters CamParam;
};

#endif
