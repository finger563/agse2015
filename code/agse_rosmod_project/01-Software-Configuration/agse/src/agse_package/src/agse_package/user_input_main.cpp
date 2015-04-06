#include "ros/ros.h"
#include <cstdlib>
#include <string.h>

// Required for boost::thread
#include <boost/thread.hpp>

// Include all components this actor requires
#include "agse_package/user_input_controller.hpp" 
#include "agse_package/user_input_imager.hpp" 


void componentThread(Component* compPtr)
{
    compPtr->startUp();
    compPtr->processQueue();
}

int main(int argc, char **argv)
{
    std::string nodeName = "user_input";
    std::string hostName = "localhost";

    for(int i = 0; i < argc; i++)
    {
        if(!strcmp(argv[i], "-nodename"))
            nodeName = argv[i+1];
	if(!strcmp(argv[i], "-hostname"))
	    hostName = argv[i+1];
    }

    ros::init(argc, argv, nodeName.c_str());

    // Create Node Handle
    ros::NodeHandle n;

    // Create Component Objects
    user_input_controller user_intput_controller_i(hostName, nodeName, "user_intput_controller_i", argc, argv); 
    user_input_imager user_input_imager_i(hostName, nodeName, "user_input_imager_i", argc, argv); 

    // Create Component Threads
    boost::thread user_intput_controller_i_thread(componentThread, &user_intput_controller_i);
    ROS_INFO("Node user_input has started user_intput_controller_i");
    boost::thread user_input_imager_i_thread(componentThread, &user_input_imager_i);
    ROS_INFO("Node user_input has started user_input_imager_i");


    ROS_INFO_STREAM("NodeMain thread id = " << boost::this_thread::get_id());

    // Create Component Threads
    user_intput_controller_i_thread.join();
    user_input_imager_i_thread.join();


    return 0; 
}

