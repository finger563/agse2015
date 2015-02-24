#include "agse_package/Component.hpp"

// required for clean shutdown when process is killed
Component::~Component()
{
    compQueue.disable();
    initOneShotTimer.stop();
}

void Component::Init(const ros::TimerEvent& event)
{
}

void Component::processQueue()
{  
    ros::NodeHandle nh;
    while (nh.ok())
    {
	this->compQueue.callAvailable(ros::WallDuration(0.01));
    }
}