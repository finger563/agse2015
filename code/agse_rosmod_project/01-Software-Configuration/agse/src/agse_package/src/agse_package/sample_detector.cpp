/*
 * Image Processor Class Definition
 * 
 * Author: Pranav Srinivas Kumar
 * Date: 2014.11.26
 */

#include "agse_package/sample_detector.hpp"

// Filter Global Variables
int hue_min = 0;
int hue_max = 255;
int saturation_min = 0;
int saturation_max = 255;
int value_min = 0;
int value_max = 255;

// Global callback function for sliders
void slider_update(int, void*){}

void Sample_Detector::create_filter_knobs_slider(){
  namedWindow("Filter Knobs", 0);    
  createTrackbar( "Hue_MIN", "Filter Knobs", &hue_min, hue_max, slider_update);
  createTrackbar( "Hue_MAX", "Filter Knobs", &hue_max, hue_max, slider_update);
  createTrackbar( "Sat_MIN", "Filter Knobs", &saturation_min, saturation_max, slider_update);
  createTrackbar( "Sat_MAX", "Filter Knobs", &saturation_max, saturation_max, slider_update);
  createTrackbar( "Value_MIN", "Filter Knobs", &value_min, value_max, slider_update);
  createTrackbar( "Value_MAX", "Filter Knobs", &value_max, value_max, slider_update);
}

// Setup Webcam Feed
void Sample_Detector::setup_webcam_feed(int cam_id){
  // Start up webcam
  webcam.open(cam_id);

  //set Height and Width of Feed
  webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
}

// Show Output Windows
void Sample_Detector::show_windows(){
  imshow("Webcam", webcam_feed);
  imshow("HSV", HSV);
  imshow("Filter", filtered_output);
  imshow("Object Tracker", tracker_output);
}

// Check for User Input
void Sample_Detector::handle_user_input(){
  // Briefly wait for key press
  key_press = cvWaitKey(20);

  // If user presses ESC, quit loop
  if (key_press == 27)
    quit = true;
}

// Main Initialize Function
void Sample_Detector::init(){
  setup_webcam_feed(0);
  create_filter_knobs_slider();
}

// Main Real-Time Loop
void Sample_Detector::run(std::vector<unsigned char> & camera_image, 
			  int widht, 
			  int height,
			  const char* fname)
{
  Mat image_rgb = Mat(480, 640, CV_8UC3, camera_image.data());
  Mat image = Mat(image_rgb.rows, image_rgb.cols, CV_8UC3);
  int from_to[] = {0, 2, 1, 1, 2, 0};
  mixChannels(&image_rgb, 1, &image, 1, from_to, 3);

  std::cout << "Before BGR to HSV Translation" << std::endl;

   // Convert from RGB TO HSV space
  cvtColor(image, HSV, COLOR_BGR2HSV);

  std::cout << "After BGR TO HSV Translation" << std::endl;
	
  // Filter HSV Image based on slider values
  inRange(HSV,
	  Scalar(hue_min, saturation_min, value_min), 
	  Scalar(hue_max, saturation_max, value_max),
	  filtered_output);
	
  // Erode and Dilate
  object_tracker.filter(filtered_output);
	
  // Track Object
  tracker_output = object_tracker.track(image, filtered_output);
	
  int nameLen = 0;
  if ( (nameLen = strlen(fname)) > 0 )
    {
      char rawName[nameLen + 10];
      char filteredName[nameLen + 20];
      sprintf(rawName,"%s.png",fname);
      sprintf(filteredName,"%s_filtered.png",fname);
      cv::imwrite(rawName,image);
      cv::imwrite(filteredName, filtered_output);
    }

  // Show all output windows
  // show_windows();
	
  // Handle user input
  handle_user_input();
}
