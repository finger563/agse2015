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

  try {
    CamParam.resize(Size(height,width));
    Mat image = Mat(height,width, CV_8UC3, raw_image_data.data());
    // Mat image = Mat(rawImage.rows, rawImage.cols, CV_8UC3);
    // int from_to[] = { 0,2, 1,1, 2,0};
    // mixChannels(&rawImage, 1, &image, 1, from_to, 3);

    MDetector.detect(image, Markers, CamParam, MarkerSize);

    //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++) {
      Markers[i].draw(image,Scalar(0,0,255),2);
    }
    //draw a 3d cube in each marker if there is 3d info
    if (  CamParam.isValid() && MarkerSize!=-1)
      for (unsigned int i=0;i<Markers.size();i++) {
	CvDrawingUtils::draw3dCube(image,Markers[i],CamParam);
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
