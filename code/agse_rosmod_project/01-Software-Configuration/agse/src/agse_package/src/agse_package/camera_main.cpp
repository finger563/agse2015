#include "ros/ros.h"
#include <cstdlib>

// Required for boost::thread
#include <boost/thread.hpp>

// Include all components this actor requires
#include "agse_package/image_processor.hpp" 


void componentThread(Component* compPtr)
{
    compPtr->startUp();
    compPtr->processQueue();
}

int main(int argc, char **argv)
{
    std::string nodeName = "camera";
    ros::init(argc, argv, nodeName.c_str());

    // Create Node Handle
    ros::NodeHandle n;

    // Create Component Objects
    image_processor image_processor_i; 

    // Create Component Threads
    boost::thread image_processor_i_thread(componentThread, &image_processor_i);
    ROS_INFO("Node camera has started image_processor_i");


    ROS_INFO_STREAM("NodeMain thread id = " << boost::this_thread::get_id());

    // Create Component Threads
    image_processor_i_thread.join();


    return 0; 
}

