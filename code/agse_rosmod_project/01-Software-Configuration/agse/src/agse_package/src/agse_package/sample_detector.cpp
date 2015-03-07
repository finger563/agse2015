/*
 * Image Processor Class Definition
 * 
 * Author: Pranav Srinivas Kumar
 * Date: 2014.11.26
 */

#include "agse_package/sample_detector.hpp"

int sensitivity = 120;

// Filter Global Variables
int hue_min = 0;
int hue_max = 255;
int saturation_min = 0;
int saturation_max = sensitivity;
int value_min = 255-sensitivity;
int value_max = 255;

int min_grayscale_thresh = 120;
int max_grayscale_thresh = 160;

Size hsv_erode_size = Size(10, 10); 
Size hsv_dilate_size = Size(25, 25); 

Size grayscale_erode_size = Size(15, 15);
Size grayscale_dilate_size = Size(50, 50);

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
}

Mat image;
Mat hsv_image;
Mat hsv_filtered_image;
Mat hsv_tracked_image;
Object_Tracker obj_tracker;

Mat grayscale_image;
Mat grayscale_filtered_image;
Mat grayscale_tracked_image;

void hsv_method(Mat &image) {

  std::cout << "SAMPLE_DETECTOR::Starting HSV METHOD" << std::endl;

   // Convert from RGB TO HSV space
  cvtColor(image, hsv_image, COLOR_BGR2HSV);

  // Filter HSV Image based on slider values
  inRange(hsv_image,
	  Scalar(hue_min, saturation_min, value_min), 
	  Scalar(hue_max, saturation_max, value_max),
	  hsv_filtered_image);

  // Erode and Dilate
  obj_tracker.filter(hsv_filtered_image, hsv_erode_size, hsv_dilate_size);
	
  cv::imwrite("Sample-02-HSV-Filtered.png", hsv_filtered_image);

  std::cout << "SAMPLE_DETECTOR::Completed HSV METHOD" << std::endl;

}

void grayscale_method(Mat& image) {

  std::cout << "SAMPLE_DETECTOR::Starting Grayscale METHOD" << std::endl;

  cvtColor(image, grayscale_image, CV_BGR2GRAY);
  cv::imwrite("Sample-04-Grayscale.png", grayscale_image);

  threshold(grayscale_image, grayscale_filtered_image, min_grayscale_thresh, max_grayscale_thresh, 0);
  cv::imwrite("Sample-05-Grayscale-Threshold.png", grayscale_filtered_image);

  // Erode and Dilate
  obj_tracker.filter(grayscale_filtered_image, grayscale_erode_size, grayscale_dilate_size);
	
  cv::imwrite("Sample-06-Grayscale-Filtered.png", grayscale_filtered_image);

  std::cout << "SAMPLE_DETECTOR::Completed Grayscale METHOD" << std::endl;

}

// Main Real-Time Loop
DetectedObject Sample_Detector::run( Mat& image, 
				     Mat& maskOutput,
				     const char* fname)
{
  DetectedObject object;
  object.state = HIDDEN;

  hsv_method(image);
  grayscale_method(image);

  Mat AND_image;
  bitwise_and(hsv_filtered_image, grayscale_filtered_image, AND_image);
  cv::imwrite("Sample-07-Bitwise-AND-Filtered.png", AND_image);

  Mat bitwise_and_tracked;
  std::cout << "Before Tracking" << std::endl;
  vector<RotatedRect> tracked_objects = obj_tracker.track(image, AND_image, maskOutput);
  std::cout << "Done Tracking" << std::endl;
  cv::imwrite("Sample-08-Bitwise-AND-Tracked.png", maskOutput);

  if ( tracked_objects.size() > 0 )
    {
      object.state = DETECTED;
      object.x = tracked_objects[0].center.x;
      object.y = tracked_objects[0].center.y;
      object.angle = tracked_objects[0].angle;
    }

  return object;
}
