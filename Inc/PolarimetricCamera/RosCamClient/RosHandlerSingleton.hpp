#ifndef __ROS_HANDLER_SINGLETON_HPP__
#define __ROS_HANDLER_SINGLETON_HPP__

// Custom includes

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>


/**
 * @brief RosHandlerSingleton: Since ROS requires a single ROS::init in the entire
 * software, and the node handle too, we created a singleton to ease this situation.
 * When the single constructor is created, the ros init is called, and the ros
 * node is initialized. Additionally, the Image transport class is created.
 * When the instance is released, ROS is shutdown, and the node handle is deleted.
*/
class RosHandlerSingleton
{
public:
    /**
     * Singletons should not be cloneable.
     */
    RosHandlerSingleton(RosHandlerSingleton&) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const RosHandlerSingleton&) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static RosHandlerSingleton* getInstance()
    {
        static bool s_firstTime = true;
        if(_instance == nullptr){
            if (s_firstTime)
            {
                int argc = 0;
                char** argv = nullptr;
                ros::init(argc, argv, "ROS_Camera_Client_node");
                s_firstTime = false;
            }
            if (ros::master::check())
            {
                _instance = new RosHandlerSingleton();
            }
        }
        return  _instance;
    }

    /**
     * @brief getNodeHandler: This is a useful class, to get the single instance
     * of the node handler. Since it can be accessed only through an instance, and
     * there is a single instance of this class, then we do not need a singleton
     * accessor
    */
    ros::NodeHandle* getNodeHandler()
    {
        return _nh;
    }

    /**
     * @brief getImageTransport: This is a useful class, to get the single instance
     * of the image transport object. Since it can be accessed only through an instance, and
     * there is a single instance of this class, then we do not need a singleton
     * accessor
    */
    image_transport::ImageTransport* getImageTransport()
    {
        return _it;
    }

    /**
     * @brief releaseSingleton: Function to delete the singleton.
    */
    static void releaseSingleton()
    {
        if(_instance)
        {
            delete _instance;
            _instance = nullptr;
        }
    }

private:
    ~RosHandlerSingleton();
    RosHandlerSingleton();
    static RosHandlerSingleton* _instance;
    static ros::NodeHandle* _nh;
    static image_transport::ImageTransport* _it;
};
#endif // __ROS_HANDLER_SINGLETON_HPP__