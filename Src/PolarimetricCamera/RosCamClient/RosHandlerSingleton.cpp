// Custom includes
#include "PolarimetricCamera/RosCamClient/RosHandlerSingleton.hpp"

// ROS includes

RosHandlerSingleton* RosHandlerSingleton::_instance = nullptr;
ros::NodeHandle* RosHandlerSingleton::_nh = nullptr;
image_transport::ImageTransport* RosHandlerSingleton::_it = nullptr;

RosHandlerSingleton::RosHandlerSingleton()
{
    // TODO: Place a timer or a trials counter to avoid hanging
    _nh = new ros::NodeHandle;
    _it = new image_transport::ImageTransport(*_nh);
}

RosHandlerSingleton::~RosHandlerSingleton()
{
    // TODO: If we include this delete, the program crashes saying:
    // "Attempt to unload library that class_loader is unaware of."
    // My guess: Since the argument is the NodeHandle as reference, it is the
    // NodeHandle who deletes this object, so we do this procedure twice.
    // delete _it;
    ros::shutdown();
    while (ros::ok())
    {
        ros::spin();
    }
    delete _nh;
}