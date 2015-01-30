#include "ros/ros.h"
#include <cstdlib>

// Required for boost::thread
#include <boost/thread.hpp>

// Include all components this actor requires
#include "agse_package/gripper_controller.hpp" 


void componentThread(Component* compPtr)
{
    compPtr->startUp();
    compPtr->processQueue();
}

int main(int argc, char **argv)
{
    std::string nodeName = "gripper";
    ros::init(argc, argv, nodeName.c_str());

    // Create Node Handle
    ros::NodeHandle n;

    // Create Component Objects
    gripper_controller gripper_controller_i; 

    // Create Component Threads
    boost::thread gripper_controller_i_thread(componentThread, &gripper_controller_i);
    ROS_INFO("Node gripper has started gripper_controller_i");


    ROS_INFO_STREAM("NodeMain thread id = " << boost::this_thread::get_id());

    // Create Component Threads
    gripper_controller_i_thread.join();


    return 0; 
}

