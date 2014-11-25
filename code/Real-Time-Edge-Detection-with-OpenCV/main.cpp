/*
 * OpenCV-based Real-time Canny Edge Detector:
 * ------------------------------------------
 * Periodically fetches webcam feed
 * Applies the canny edge detector to a queried image frame 
 * Displays the webcam feed and processed webcam feed in separate windows
 * Low and high threshold sliders that decide the inclusion or rejection of detected edge
 * Press Escape to quit application 
 *
 * Author: Pranav Srinivas Kumar
 * Date: 2014.11.25
 */

#include <iostream>
#include "Canny_Edge_Detector.h"

using namespace std;

int main(int argc, char** argv)
{
    // Canny Edge Detector Object
    Canny_Edge_Detector CED;

    // Initialize webcam, output windows and variables
    CED.init();

    // Run RT Loop
    CED.run();
 
    // Clean up on exit
    CED.cleanup();
}
