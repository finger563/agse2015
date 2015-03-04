/*
 * Object Tracker Class Declaration
 * 
 * Author: Pranav Srinivas Kumar
 * Date: 2014.11.26
 */

#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace cv;

class Object_Tracker{

public:
	// Constructor
	Object_Tracker();

	// Image Filter 
	void filter(Mat &filter_matrix);

	// Main Object Tracker Function
	Mat track(Mat webcam_feed, Mat filterd_output);

        // Get Center Point of Detected Object
        Vector<Point2f> get_object_center();

        // Get Angle of Detected Object
        float get_object_angle();

        // Set minimum area
        void set_min_area(int this_area);

        // Set maximum area
        void set_max_area(int this_area);

        // Set max objects
	void set_max_objects(int this_number);

private:
	int max_objects;
	int min_area;
	int max_area;

        Vector<Point2f> object_center;
        float object_angle;
};
