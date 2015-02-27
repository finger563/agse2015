#include "ros/ros.h"
#include <cstdlib>
#include <string.h>

// Required for boost::thread
#include <boost/thread.hpp>

// Include all components this actor requires
#include "agse_package/image_sensor.hpp" 


void componentThread(Component* compPtr)
{
    compPtr->startUp();
    compPtr->processQueue();
}

int main(int argc, char **argv)
{
    std::string nodeName = "imager";

    for(int i = 0; i < argc; i++)
        if(!strcmp(argv[i], "-nodename"))
            nodeName = argv[i+1];

    ros::init(argc, argv, nodeName.c_str());

    // Create Node Handle
    ros::NodeHandle n;

    // Create Component Objects
    image_sensor image_sensor_i(nodeName, argc, argv); 

    // Create Component Threads
    boost::thread image_sensor_i_thread(componentThread, &image_sensor_i);
    ROS_INFO("Node imager has started image_sensor_i");


    ROS_INFO_STREAM("NodeMain thread id = " << boost::this_thread::get_id());

    // Create Component Threads
    image_sensor_i_thread.join();


    return 0; 
}
