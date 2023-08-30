#ifndef __POLARIMETRIC_CAMERA_SERVER_HPP__
#define __POLARIMETRIC_CAMERA_SERVER_HPP__

// This include should be place at the beginning, in order to avoid compilation errors
#include <ros/ros.h>

// OpenCV includes

// STD includes
#include <string>
#include <thread>

// Qt Includes

// ROS includes
#include <cv_bridge/cv_bridge.h>

// Custom includes
#include "CameraHandler.hpp"
#include "CameraTypes.hpp"
#include "PolarimetricCamera/IPolarimetricCamera.hpp"
#include "std_srvs/Empty.h"

/**
 * @brief PolarimetricCameraServer class
 *
 *  Main ROS Server class. This class will communicate with the different connected cameras.
 *
 *  This class coordinates the initialization, and the synchronization of the captured images.
 *  This class contains two threads: the temperature publisher thread, and the free-run image publisher
 *  thread. Particularly, the last one will publish images as soon as it retrieves them from
 *  all the cameras. The publisher speed will be the speed of the slowest camera. Additionally,
 *  all the connected and initialized cameras will publish messages with the same timestamp, and
 *  sequence number. This way, it will be easy to find the corresponding paired images when
 *  more than one camera is connected.
 *
 *   There is a mode in which the image publisher thread is not enabled, and the images are
 *  send per request. In this case, a service must be called to request one image set from
 *  all the cameras. The way the images are retrieved and published is the same as explained
 *  above.
 *
 *  The camera topics and services are the ones exposed by each CameraHandler. The names
 *  are listed in the RosNaming.hpp file. The only global service this class creates
 *  is the one to request the images, and it is created only if the MODE flag in the
 *  initializeServer is set to false.
*/
class PolarimetricCameraServer
{
public:
    /**
     * @brief Constructor. This function does not initalize a ROS node, but it
     *  expects that it will be intialized outside, and this class will profite
     *  of it. This way, the server can have several classes that uses the same ROS
     *  node. In this function, all the CameraHandlers will be created.
     *
     * @arg nh: ROS node handler pointer, already initialized.
     * @arg camerasConf: Struct with all the cameras configuration (name, driver, and master / slave mode).
    */
    PolarimetricCameraServer(ros::NodeHandle *nh, std::vector<CameraConfig> camerasConf);
    ~PolarimetricCameraServer();

    /**
     * @brief initializeServer: Initialize all the CameraHandlers, the service to
     *   request an image (if required), and the threads.
     *
     * @arg mode: If true, the camera is initialized in continuous capture mode. In this case,
     *  the publisher thread is started, which will continuously request images from the cameras
     *  and publish them. If false, it is initialized in single image mode, in which an image
     *  is published from each camera only when the corresponding service is called.
     * @arg loopFreq: Frequency of the continous mode loop. If mode is false, this
     *  parameter is not used.
     * @arg bitDepth: Camera bit-depth to set. For now, only 8 and 12 are
     *  allowed values.
    */
    void initializeServer(bool mode, int loopFreq, int bitDepth);

    /**
     * @brief stopThreads: It will stop the publishing threads by clearing the corresponding
     *   running flag.
    */
    void stopThreads();

private:

    /**
     * @brief startPublisherThread: This function executes the main thread of this module, when
     *   the mode is set to continuous. The loop runs at the frequency set in the function initializeServer.
     *   It will first request all the images from the cameras configured as MASTER, and then it will
     *   request the corresponding images from the SLAVE cameras. Then, once all the images retrieved,
     *   they are published in the corresponding thread.
    */
    void startPublisherThread();

    /**
     * @brief publishImages: It first reads the last captured images from all the cameras,
     *   and then it publishes them into the corresponding topics. The MASTER cameras are
     *   read in first step, and then the SLAVE cameras are read in the second step.
     *   All the images are published with the same timestamp and sequence number. The
     *   images are always retrieved, but they are published only if there are clients
     *   subscribed to the topics.
    */
    void publishImages();

    /**
     * @brief publishTemperature: Thread to publish the read temperatures from all the cameras.
     *   They are published only if there are clients subscribed to the corresponding topic.
    */
    void publishTemperature();

    /**
     * @brief requestImageServiceCallback: Service callback. It forces the publication of
     *    the last captured image by all the cameras.
     *
     * @returns True if no error occurs.
    */
    bool requestImageServiceCallback(
        std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res);

    // We avoid variable optimization from the compiler
    volatile bool _isThreadRunning;
    ros::NodeHandle* _nodeHandler;

    std::string _camReqImageServiceName;
    std::vector<std::unique_ptr<CameraHandler>> _camHandlers;
    std::vector<int> _masterCamIdx;
    std::vector<int> _slaveCamIdx;

    // ROS services
    ros::ServiceServer _requestCameraImgSrv;

    /// Internal variables
    std_msgs::Header _imgMsgHeader;
    cv_bridge::CvImage _imgBridge;
    int _loopFreq;
    std::string _chosenImgEncoding;

    // Threads objects
    std::thread _freeRunThread;
    std::thread _tempPublisherThread;
};

#endif // __POLARIMETRIC_CAMERA_SERVER_HPP__
