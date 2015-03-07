#include "agse_package/payloadbay_detector.hpp"

void PayloadBay_Detector::init(float msize, const char* camParamFileName)
{
  MarkerSize = msize;
  CamParam.readFromXMLFile(camParamFileName);
}

DetectedObject PayloadBay_Detector::run( Mat& image, 
					 Mat& maskOutput,
					 const char* fname)
{
  DetectedObject object;
  object.state = HIDDEN;
  std::vector<Marker> Markers;
  std::vector<Point2f> Centers;
  Point2f center(0.0f,0.0f);
  float angle = 0.0f;
  try {
    float width = image.cols;
    float height = image.rows;
    CamParam.resize(Size(height,width));

    MDetector.detect(image, Markers, CamParam, MarkerSize);
    //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++) {
      Markers[i].draw(maskOutput,Scalar(0,0,255),4);
    }
    //draw a 3d cube in each marker if there is 3d info
    if (  CamParam.isValid() && MarkerSize!=-1)
      for (unsigned int i=0;i<Markers.size();i++) {
	CvDrawingUtils::draw3dAxis(maskOutput,Markers[i],CamParam);
      }

    if (Markers.size() > 0) {
      for (unsigned int i=0;i<Markers.size();i++) {
	Centers.push_back(Markers[i].getCenter());
	center += Centers[i];
	if (i>0)
	  angle += atan2(Centers[i].y-Centers[i-1].y,Centers[i].x-Centers[i-1].x);
      }
      angle = angle/((float)(Centers.size()-1)) * 180.0f / M_PI;
      center = center * (1.0f / (float)Centers.size());

      Scalar color = Scalar(255, 
			    255, 
			    255);
      circle(image,center,20.0f,color,-1);

      object.state = DETECTED;
      if (Markers.size() == 1)
	object.state = PARTIAL;
      object.x = center.x;
      object.y = center.y;
      object.angle = angle;
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
      object.state = HIDDEN;
    }
  return object;
}
