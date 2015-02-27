#include "ros/ros.h"
#include <cstdlib>
#include <string.h>

// Required for boost::thread
#include <boost/thread.hpp>

// Include all components this actor requires
#include "agse_package/radial_actuator_controller.hpp" 
#include "agse_package/vertical_actuator_controller.hpp" 
#include "agse_package/servo_controller.hpp" 


void componentThread(Component* compPtr)
{
    compPtr->startUp();
    compPtr->processQueue();
}

int main(int argc, char **argv)
{
    std::string nodeName = "positioning";

    for(int i = 0; i < argc; i++)
        if(!strcmp(argv[i], "-nodename"))
            nodeName = argv[i+1];

    ros::init(argc, argv, nodeName.c_str());

    // Create Node Handle
    ros::NodeHandle n;

    // Create Component Objects
    radial_actuator_controller radial_controller_i(nodeName, argc, argv); 
    vertical_actuator_controller vertical_controller_i(nodeName, argc, argv); 
    servo_controller servo_controller_i(nodeName, argc, argv); 

    // Create Component Threads
    boost::thread radial_controller_i_thread(componentThread, &radial_controller_i);
    ROS_INFO("Node positioning has started radial_controller_i");
    boost::thread vertical_controller_i_thread(componentThread, &vertical_controller_i);
    ROS_INFO("Node positioning has started vertical_controller_i");
    boost::thread servo_controller_i_thread(componentThread, &servo_controller_i);
    ROS_INFO("Node positioning has started servo_controller_i");


    ROS_INFO_STREAM("NodeMain thread id = " << boost::this_thread::get_id());

    // Create Component Threads
    radial_controller_i_thread.join();
    vertical_controller_i_thread.join();
    servo_controller_i_thread.join();


    return 0; 
}

