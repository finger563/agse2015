#include "agse_package/payloadbay_detector.hpp"

void PayloadBay_Detector::init(float msize, const char* camParamFileName)
{
  MarkerSize = msize;
  CamParam.readFromXMLFile(camParamFileName);
}

void PayloadBay_Detector::run(std::vector<unsigned char> & raw_image_data, 
					     int width, 
					     int height, 
					     const char* fname)
{
  std::vector<Marker> Markers;
  std::vector<Point2f> Centers;
  Point2f center(0.0f,0.0f);
  try {
    CamParam.resize(Size(height,width));
    Mat image = Mat(height,width, CV_8UC3, raw_image_data.data());

    MDetector.detect(image, Markers, CamParam, MarkerSize);
    for (unsigned int i=0;i<Markers.size();i++) {
      Centers.push_back(Markers[i].getCenter());
      center += Centers[i];
    }
    center = center * (1.0f / (float)Centers.size());
    Scalar color = Scalar(255, 
			  255, 
			  255);
    circle(image,center,20.0f,color,-1);
    //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++) {
      Markers[i].draw(image,Scalar(0,0,255),2);
    }
    //draw a 3d cube in each marker if there is 3d info
    if (  CamParam.isValid() && MarkerSize!=-1)
      for (unsigned int i=0;i<Markers.size();i++) {
	CvDrawingUtils::draw3dAxis(image,Markers[i],CamParam);
      }

    int nameLen = 0;
    if ( (nameLen = strlen(fname)) > 0 )
      {
	char rawName[nameLen + 10];
	char edgeName[nameLen + 20];
	sprintf(rawName,"%s.png",fname);
	sprintf(edgeName,"%s_edges.png",fname);
	cv::imwrite(rawName,image);
	cv::imwrite(edgeName, MDetector.getThresholdedImage());
      }
  } catch (std::exception &ex)
    {
      printf("ERROR: caught exception detecting markers: %s\n",ex.what());
    }
}
