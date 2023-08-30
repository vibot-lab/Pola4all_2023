// OpenCV includes

// STD includes

// Qt Includes

// Custom includes
#include "CameraTypes.hpp"
#include "PolarimetricCameraServer.hpp"
#include "RosNaming.hpp"

// ROS includes
#include <sensor_msgs/image_encodings.h>

PolarimetricCameraServer::PolarimetricCameraServer(ros::NodeHandle *nh, std::vector<CameraConfig> camerasConf) :
    _isThreadRunning(false),
    _nodeHandler(nh),
    _camReqImageServiceName(CAMERA_REQUEST_CAMERA_IMAGE_SERVICE),
    _camHandlers(camerasConf.size()),
    _loopFreq(70),
    _chosenImgEncoding("")
{
    assert(camerasConf.size());
    for (size_t i = 0; i < camerasConf.size(); i++)
    {
        const CameraConfig& conf = camerasConf[i];
        _camHandlers[i].reset(new CameraHandler(_nodeHandler, conf.camDriver, conf.camName, conf.isMaster));
        if (conf.isMaster)
        {
            _masterCamIdx.push_back(i);
        }
        else
        {
            _slaveCamIdx.push_back(i);
        }
    }
}

PolarimetricCameraServer::~PolarimetricCameraServer()
{
    stopThreads();
}

void PolarimetricCameraServer::initializeServer(bool mode, int loopFreq, int bitDepth)
{
    ROS_INFO("PolarimetricCameraServer: Initializing server...");
    _loopFreq = loopFreq;

    for (std::unique_ptr<CameraHandler>& handler : _camHandlers)
    {
        handler->initializeTopics(bitDepth);
    }

    if (bitDepth == 8)
    {
        _chosenImgEncoding = sensor_msgs::image_encodings::MONO8;
    }
    else if (bitDepth == 12)
    {
        _chosenImgEncoding = sensor_msgs::image_encodings::MONO16;
    }
    else
    {
        std::cout << "Unrecognized bit-depth: " << bitDepth << std::endl;
        assert(0);
    }
    _imgMsgHeader.seq = 0;
    ROS_INFO("PolarimetricCameraServer: Server initialized!!!");
    _isThreadRunning = true;

    _tempPublisherThread = std::thread(&PolarimetricCameraServer::publishTemperature, this);

    if(mode)
    {
        ROS_WARN("PolarimetricCameraServer: Initializing camera in free-run mode");
        _freeRunThread = std::thread(&PolarimetricCameraServer::startPublisherThread, this);
    }
    else
    {
        ROS_WARN("PolarimetricCameraServer: Initializing camera in trigger mode");
        _requestCameraImgSrv = _nodeHandler->advertiseService(
            _camReqImageServiceName.c_str(),
            &PolarimetricCameraServer::requestImageServiceCallback,
            (PolarimetricCameraServer*)this);
    }
}

bool PolarimetricCameraServer::requestImageServiceCallback(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
{
    publishImages();
    return true;
}

void PolarimetricCameraServer::publishImages()
{
    std::vector<cv::Mat> imgsRetrieved;
    std::vector<int> publicationOrder;

    for (const int &camIdx : _masterCamIdx)
    {
        cv::Mat img = _camHandlers[camIdx]->getImage();

        if (!img.empty())
        {
            imgsRetrieved.push_back(img.clone());
            publicationOrder.push_back(camIdx);
        }
    }
    for (const int &camIdx : _slaveCamIdx)
    {
        cv::Mat img = _camHandlers[camIdx]->getImage();
        if (!img.empty())
        {
            imgsRetrieved.push_back(img.clone());
            publicationOrder.push_back(camIdx);
        }
    }

    // We use the same sequence number and timestamp for all the published images
    _imgMsgHeader.seq += 1;
    _imgMsgHeader.stamp = ros::Time::now();

    for (size_t i = 0; i < imgsRetrieved.size(); i++)
    {
        if (_camHandlers[publicationOrder[i]]->isImageSubscribed())
        {
            sensor_msgs::Image sensorImgMsg;
            _imgBridge = cv_bridge::CvImage(_imgMsgHeader,
                _chosenImgEncoding,
                imgsRetrieved[i]);

            // The camera is in 12bits modes, and storing the image into 16 bits size
            _imgBridge.toImageMsg(sensorImgMsg); // from cv_bridge to sensor_msgs::Image
            _camHandlers[publicationOrder[i]]->publishImg(sensorImgMsg);
        }
    }
}

void PolarimetricCameraServer::publishTemperature()
{
    ros::Rate publisherFreq(10);
    while(ros::ok() && _isThreadRunning)
    {
        for (const std::unique_ptr<CameraHandler>& handler : _camHandlers)
        {
            if (handler->isTemperatureSubscribed())
            {
                std_msgs::Float32 tempMsg;
                tempMsg.data = handler->getTemperature();
                handler->publishTemp(tempMsg);
            }
        }
        publisherFreq.sleep();
    }
}

void PolarimetricCameraServer::startPublisherThread()
{
    ros::Rate idleTimer(10);
    ros::Rate publisherTimer(_loopFreq);

    // Start free run mode
    ROS_INFO("PolarimetricCameraServer: Initializing Camera Free Run mode");
    while (ros::ok() && _isThreadRunning)
    {
        publishImages();
        publisherTimer.sleep();
    }

    for (const std::unique_ptr<CameraHandler>& handler : _camHandlers)
    {
        handler->stopGrabbing();
    }
}

void PolarimetricCameraServer::stopThreads()
{
    // We first stop the process of capturing images to avoid double initialization.
    _isThreadRunning = false;
    if (_freeRunThread.joinable())
    {
        ROS_INFO("Joining camera thread");
        _freeRunThread.join();
    }
    if (_tempPublisherThread.joinable())
    {
        ROS_INFO("Joining temperature thread");
        _tempPublisherThread.join();
    }
    _imgMsgHeader.seq = 0;
}
