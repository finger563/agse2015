/*
 * Canny Edge Detector Class Definition
 * 
 * Author: Pranav Srinivas Kumar
 * Date: 2014.11.25
 */

#include "Canny_Edge_Detector.h"

int low_slider_position = 0;
int high_slider_position = 200;

// Callback to update low_slider_position when slider is moved
void low_slider_update(int slider_value){
    low_slider_position = slider_value;
}

// Callback to update high_slider_position when slider is moved
void high_slider_update(int slider_value){
    high_slider_position = slider_value;
}

// Canny Edge Detector Constructor
Canny_Edge_Detector::Canny_Edge_Detector(){
    low_threshold_max = 2048;
    high_threshold_max = 2048;
}

// Initialize Edge Detector
void Canny_Edge_Detector::init(){
    // Create GUI Windows
    create_windows();

    // Setup webcam feed
    setup_webcam_feed(0);

    // Setup frames
    setup_frames();
}

// Main Edge Detection Function
IplImage * Canny_Edge_Detector::detect(){
    // Convert source frame to greyscale version
    cvCvtColor(input_frame, temp_frame, CV_RGB2GRAY);
	
    // Perform canny edge detection
    cvCanny(temp_frame, temp_frame, low_slider_position, high_slider_position, 3);

    // Pass back our now processed frame!
    return temp_frame;	
}

// Real-Time Loop
void Canny_Edge_Detector::run(){
    char key_press;
    bool quit = false;
    
    while (quit == false)
    {
	// Fetch input frame from webcam feed
	input_frame = cvQueryFrame(webcam_feed);

	// Show webcam image
        cvShowImage("Webcam", input_frame);
        
        // Call the canny edge detector to process new webcam frame
        output_frame = detect();

	// Show processed image
        cvShowImage("Canny Edge Detector", output_frame);

	// Briefly wait for key press
        key_press = cvWaitKey(20);

	// If user presses ESC, quit loop
        if (key_press == 27)
             quit = true;
    }
}

// Create Webcam window and Edge Detector output window
void Canny_Edge_Detector::create_windows(){
    cvNamedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Canny Edge Detector", CV_WINDOW_AUTOSIZE);
    // Create slider on edge detector window to obtain real-time user input
    cvCreateTrackbar("Low Threshold", "Canny Edge Detector", &low_slider_position, low_threshold_max, low_slider_update);
    cvCreateTrackbar("High Threshold", "Canny Edge Detector", &high_slider_position, high_threshold_max, high_slider_update);
}

// Setup Webcam feed with cam_id
void Canny_Edge_Detector::setup_webcam_feed(int cam_id){
    webcam_feed = cvCaptureFromCAM(cam_id);
}

// Setup Edge detector frames
void Canny_Edge_Detector::setup_frames(){
    // Create an image from frame capture
    input_frame = cvQueryFrame(webcam_feed);

    // Create a greyscale image which is the size of our captured image
    output_frame = cvCreateImage(cvSize(input_frame->width, input_frame->height), IPL_DEPTH_8U, 1);

    // Create a frame to use as our temporary copy of the current frame but in grayscale mode
    temp_frame = cvCreateImage(cvSize(input_frame->width, input_frame->height), IPL_DEPTH_8U, 1);
}

// Clean up Image frames, webcam feed and output windows
void Canny_Edge_Detector::cleanup(){
    cvReleaseImage(&input_frame);
    cvReleaseImage(&output_frame);
    temp_frame = NULL;
    cvReleaseImage(&temp_frame);
    cvReleaseCapture(&webcam_feed);
    cvDestroyAllWindows();
}
