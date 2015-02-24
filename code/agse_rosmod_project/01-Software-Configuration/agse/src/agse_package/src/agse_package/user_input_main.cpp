#include "ros/ros.h"
#include <cstdlib>

// Required for boost::thread
#include <boost/thread.hpp>

// Include all components this actor requires
#include "agse_package/user_input_controller.hpp" 


void componentThread(Component* compPtr)
{
    compPtr->startUp();
    compPtr->processQueue();
}

int main(int argc, char **argv)
{
    std::string nodeName = "user_input";
    ros::init(argc, argv, nodeName.c_str());

    // Create Node Handle
    ros::NodeHandle n;

    // Create Component Objects
    user_input_controller user_intput_controller_i; 

    // Create Component Threads
    boost::thread user_intput_controller_i_thread(componentThread, &user_intput_controller_i);
    ROS_INFO("Node user_input has started user_intput_controller_i");


    ROS_INFO_STREAM("NodeMain thread id = " << boost::this_thread::get_id());

    // Create Component Threads
    user_intput_controller_i_thread.join();


    return 0; 
}

