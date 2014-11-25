/*
 * Canny Edge Detector Class Declaration
 * 
 * Author: Pranav Srinivas Kumar
 * Date: 2014.11.25
 */

#include "opencv/cv.h"
#include "opencv/highgui.h"

class Canny_Edge_Detector {

public:
	// Constructor
	Canny_Edge_Detector();

	// Setup GUI Windows, Webcam feed and frame variables
        void init();

	// Edge Detection function
        IplImage * detect();

	// Main Real-Time Loop
        void run();

	// Create Windows for Webcam feed and Edge Detector output
	void create_windows();

	// Setup Webcam feed variable
        void setup_webcam_feed(int cam_id=0);

	// Setup frame variables 
        void setup_frames();

	// Cleanup frame variables and windows
	void cleanup();

private:
	// Input Frame
	IplImage * input_frame;

	// Processed Frame
        IplImage * output_frame;

	// Temp Frame used during processing	
        IplImage * temp_frame;

	// Max value of lower threshold
	// Any edge below this threshold is rejected
	int low_threshold_max;

	// Max value of higher threshold
	// Any edge above this threshold is rejected
	int high_threshold_max;

	// Webcam feed
	CvCapture * webcam_feed;
};
