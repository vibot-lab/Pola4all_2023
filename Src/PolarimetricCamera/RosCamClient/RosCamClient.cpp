// OpenCV includes

// STD includes
#include <condition_variable>
#include <mutex>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/RosCamClient/RosCamClient.hpp"
#include "RosNaming.hpp"

// ROS includes
#include <cv_bridge/cv_bridge.h>
#include <polarimetric_camera_server/CameraParameters.h>
#include <polarimetric_camera_server/ParameterRequested.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

static std::mutex s_imgMutex;
static std::mutex s_tempMtx;
static std::condition_variable s_cvRos;
static std::mutex s_rosInitMtx;

RosCamClient RosCamClient::instance;

RosCamClient::RosCamClient() :
    _camImgTopicName(CAMERA_IMAGE_TOPIC_NAME),
    _camStateTopicName(CAMERA_STATE_TOPIC_NAME),
    _camTempTopicName(CAMERA_TEMPERATURE_TOPIC_NAME),
    _camChangeParameterServiceName(CAMERA_CHANGE_PARAM_SERVICE),
    _camRequestParamsTopicName(CAMERA_REQUEST_PARAMS_TOPIC_NAME),
    _camIsAliveServiceName(CAMERA_IS_ALIVE_SERVICE),
    _requestSpecificParamSrvName(CAMERA_REQUEST_SPECIFIC_PARAM_SERVICE),
    _topicWaitTimeout(1.5),
    _lastRetrievedTemperature(0.0),
    _isRunning(false),
    _isRosInitialized(false),
    _topicsInitialized(false),
    _usedEncoding("")
{
    _parametersCallbacks[AUTO_EXPOSURE] = &RosCamClient::enableAutoExposure;
    _parametersCallbacks[AUTO_GAIN] = &RosCamClient::enableAutoGain;
    _parametersCallbacks[AUTO_WHITE_BALANCE] = &RosCamClient::enableAutoWhiteBalance;
    _parametersCallbacks[FRAME_RATE] = &RosCamClient::changeFrameRate;
    _parametersCallbacks[EXPOSURE_TIME] = &RosCamClient::changeExposureTime;
    _parametersCallbacks[GAIN] = &RosCamClient::changeGain;
    _parametersCallbacks[WHITE_BALANCE_GAINS] = &RosCamClient::adjustWhiteBalance;
}

RosCamClient::~RosCamClient()
{
    RosHandlerSingleton::releaseSingleton();
}

bool RosCamClient::initializeRosClient()
{
    if (!RosHandlerSingleton::getInstance())
    {
        ROS_WARN("Cannot contact the camera server");
        return false;
    }

    _isRunning = true;
    _isRosInitialized = false;

    {
        std::unique_lock<std::mutex> lck(s_rosInitMtx);
        _freeRunThread = std::thread(&RosCamClient::refreshEvents, this);

        while (!_isRosInitialized && ros::ok())
        {
            ROS_INFO("Waiting for initialization");
            s_cvRos.wait(lck);
            ros::spinOnce();
        }
    }

    ros::Rate waitRate(2);
    int connCounter = 0;
    while (_requestState.getNumSubscribers() == 0 && ros::ok())
    {
        if (connCounter > 10)
        {
            return false;
        }
        ros::spinOnce();
        waitRate.sleep();
        ROS_INFO("Waiting for server");
        connCounter++;
    }
    ROS_INFO("Server found!");

    return true;
}

void RosCamClient::callChangeParamsService(const CameraState &params, const CameraParams& paramToChange)
{
    ros::NodeHandle* clientNodeHandler = RosHandlerSingleton::getInstance()->getNodeHandler();
    ros::ServiceClient client =
        clientNodeHandler->serviceClient<polarimetric_camera_server::CameraParameters>(_camChangeParameterServiceName.c_str());
    polarimetric_camera_server::CameraParameters req;

    req.request.gain = params.gain;
    req.request.exposureUs = params.exposureUs;
    req.request.frameRate = params.frameRate;
    req.request.redGain = params.redGain;
    req.request.greenGain = params.greenGain;
    req.request.blueGain = params.blueGain;
    req.request.autoGainEnabled = params.autoGainEnabled;
    req.request.autoExposureEnabled = params.autoExposureEnabled;
    req.request.autoWhiteBalanceEnabled = params.autoWhiteBalanceEnabled;
    req.request.paramToChange = paramToChange;

    bool ret_val = client.call(req);
    if (!ret_val)
    {
        ROS_WARN("Cannot change the requested parameter");
    }
}

bool RosCamClient::initCam(int bitDepth)
{
    bool ret_val = false;
    if (bitDepth == 8)
    {
        _usedEncoding = sensor_msgs::image_encodings::MONO8;
    }
    else if (bitDepth == 12)
    {
        _usedEncoding = sensor_msgs::image_encodings::MONO16;
    }
    else
    {
        std::cerr << "Unrecognized bit depth: " << bitDepth << std::endl;
        assert(0);
    }

    if (!initializeRosClient())
    {
        deinitCam();
        return false;
    }

    if (isAlive())
    {
        emitState();
        std::unique_lock<std::mutex> lck(s_imgMutex);
        getSingleImageSynchronously(_lastRetrievedImage);
        ret_val = true;
    }
    else
    {
        deinitCam();
        ROS_WARN("Cannot communicate with the camera. Did you connect the camera into the server ?");
    }
    return ret_val;
}

void RosCamClient::emitState()
{
    if (_stateCallback)
    {
        std_msgs::Empty dummy;
        _requestState.publish(dummy);
    }
}

void RosCamClient::deinitCam()
{
    _isRunning = false;
    _isRosInitialized = false;
    if (_freeRunThread.joinable())
    {
        ROS_INFO("Joining thread");
        _freeRunThread.join();
    }

    _camStateSubs.shutdown();
    _camImagesSubscriber.shutdown();
    _temperatureSubscriber.shutdown();
    _requestState.shutdown();
    _topicsInitialized = false;

    _lastRetrievedImage = cv::Mat();
    _lastRetrievedTemperature = 0.0;
}

void RosCamClient::changeExposureTime(const CameraState &state)
{
    callChangeParamsService(state, EXPOSURE_TIME);
}

void RosCamClient::changeFrameRate(const CameraState &state)
{
    callChangeParamsService(state, FRAME_RATE);
}

void RosCamClient::changeGain(const CameraState &state)
{
    callChangeParamsService(state, GAIN);
}

void RosCamClient::adjustWhiteBalance(const CameraState &state)
{
    callChangeParamsService(state, WHITE_BALANCE_GAINS);
}

void RosCamClient::getSingleImageSynchronously(cv::Mat& img) const
{
    img = cv::Mat();
    bool imgRetrieved = false;

    while (ros::ok() && !imgRetrieved)
    {
        ros::NodeHandle* clientNodeHandler = RosHandlerSingleton::getInstance()->getNodeHandler();
        sensor_msgs::ImageConstPtr myImg = ros::topic::waitForMessage<sensor_msgs::Image>(
                _camImgTopicName.c_str(),
                *clientNodeHandler,
                _topicWaitTimeout);

        if (myImg != NULL)
        {
            cv_bridge::CvImagePtr imgPtr;
            try
            {
                imgPtr = cv_bridge::toCvCopy(*myImg, _usedEncoding);
                imgPtr->image.copyTo(img);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            imgRetrieved = true;
        }
        else
        {
            ROS_INFO("Not image received. Continuing the loop");
            ros::spinOnce();
        }
    }
}

void RosCamClient::getRawImage(cv::Mat& img) const
{
    std::unique_lock<std::mutex> lck(s_imgMutex);
    _lastRetrievedImage.copyTo(img);
}

float RosCamClient::getTemperature() const
{
    std::unique_lock<std::mutex> lck(s_tempMtx);
    return _lastRetrievedTemperature;
}

std::vector<int> RosCamClient::getGainLimits() const
{
    std::vector<int> img;
    return img;
}

std::vector<double> RosCamClient::getExposureLimits() const
{
    std::vector<double> img;
    return img;
}

bool RosCamClient::requestParam(CameraState &params, const CameraParams& requestedParam) const
{
    ros::NodeHandle* clientNodeHandler = RosHandlerSingleton::getInstance()->getNodeHandler();
    ros::ServiceClient client = clientNodeHandler->serviceClient<polarimetric_camera_server::ParameterRequested>(
        _requestSpecificParamSrvName.c_str());
    polarimetric_camera_server::ParameterRequested req;
    req.request.paramRequested = requestedParam;

    bool ret_val = client.call(req);
    if (!ret_val)
    {
        ROS_WARN("Failed in requesting frame rate");
        return false;
    }

    params.gain = req.response.gain;
    params.exposureUs = req.response.exposureUs;
    params.frameRate = req.response.frameRate;
    params.redGain = req.response.redGain;
    params.greenGain = req.response.greenGain;
    params.blueGain = req.response.blueGain;
    params.autoGainEnabled = static_cast<bool>(req.response.autoGainEnabled);
    params.autoExposureEnabled = static_cast<bool>(req.response.autoExposureEnabled);
    params.autoWhiteBalanceEnabled = static_cast<bool>(req.response.autoWhiteBalanceEnabled);

    return true;
}

int RosCamClient::getFrameRate() const
{
    CameraState params;
    if (!requestParam(params, FRAME_RATE))
    {
        ROS_WARN("Failed in requesting frame rate");
        return 0;
    }
    return params.frameRate;
}

double RosCamClient::getExposureTime() const
{
    CameraState params;
    if (!requestParam(params, EXPOSURE_TIME))
    {
        ROS_WARN("Failed in requesting exposure time");
        return 0.0;
    }
    return params.exposureUs;
}

double RosCamClient::getGain() const
{
    CameraState params;
    if (!requestParam(params, GAIN))
    {
        ROS_WARN("Failed in requesting gain");
        return 0.0;
    }
    return params.gain;
}

std::vector<double> RosCamClient::getWhiteBalanceGains() const
{
    std::vector<double> gains;
    CameraState params;
    if (requestParam(params, WHITE_BALANCE_GAINS))
    {
        gains.push_back(params.redGain);
        gains.push_back(params.greenGain);
        gains.push_back(params.blueGain);
    }
    else
    {
        ROS_WARN("Failed in requesting white balance gains");
    }

    return gains;
}

void RosCamClient::enableAutoExposure(const CameraState &state)
{
    callChangeParamsService(state, AUTO_EXPOSURE);
    if (!state.autoExposureEnabled)
    {
        emitState();
    }
}

void RosCamClient::enableAutoGain(const CameraState &state)
{
    callChangeParamsService(state, AUTO_GAIN);
    if (!state.autoGainEnabled)
    {
        emitState();
    }
}

void RosCamClient::enableAutoWhiteBalance(const CameraState &state)
{
    callChangeParamsService(state, AUTO_WHITE_BALANCE);
    if (!state.autoWhiteBalanceEnabled)
    {
        emitState();
    }
}

bool RosCamClient::isAutoGainEnabled() const
{
    CameraState params;
    if (!requestParam(params, AUTO_GAIN))
    {
        ROS_WARN("Failed in requesting if the camera is in auto-gain mode");
        return false;
    }
    return params.autoGainEnabled;
}

bool RosCamClient::isAutoExposureEnabled() const
{
    CameraState params;
    if (!requestParam(params, AUTO_EXPOSURE))
    {
        ROS_WARN("Failed in requesting if the camera is in auto-exposure mode");
        return false;
    }
    return static_cast<bool>(params.autoExposureEnabled);
}

bool RosCamClient::isAutoWhiteBalanceEnabled() const
{
    CameraState params;
    if (!requestParam(params, AUTO_WHITE_BALANCE))
    {
        ROS_WARN("Failed in requesting if the camera is in auto-white balance mode");
        return false;
    }
    return static_cast<bool>(params.autoWhiteBalanceEnabled);
}

bool RosCamClient::isAlive() const
{
    bool rosState = ros::master::check() && _topicsInitialized;
    if (rosState)
    {
        // If we cannot put the camera in free run mode, then the camera is not alive
        ros::NodeHandle* clientNodeHandler = RosHandlerSingleton::getInstance()->getNodeHandler();
        ros::ServiceClient client = clientNodeHandler->serviceClient<std_srvs::Empty>(_camIsAliveServiceName.c_str());
        std_srvs::Empty req;
        rosState = client.call(req);
    }
    return rosState;
}

void RosCamClient::changeCameraParameters(CameraParams paramToChange, CameraState state)
{
    // IMPORTANT: If you add more types to the CameraParams class, be sure
    // to add the corresponding case here
    assert(NUM_OF_PARAMS == 8);
    assert(paramToChange < NUM_OF_PARAMS);

    if (paramToChange == ALL)
    {
        for(int i = AUTO_EXPOSURE; i < NUM_OF_PARAMS; i++)
        {
            (*this.*_parametersCallbacks.at(static_cast<CameraParams>(i)))(state);
        }
    }
    else
    {
        (*this.*_parametersCallbacks.at(paramToChange))(state);
    }
}

CameraState RosCamClient::getCameraState() const
{
    std::vector<double> gains = getWhiteBalanceGains();
    CameraState stateStruct = {
        .gain = getGain(),
        .exposureUs = getExposureTime(),
        .frameRate = getFrameRate(),
        .redGain = gains[0],
        .greenGain = gains[1],
        .blueGain = gains[2],
        .autoGainEnabled = isAutoGainEnabled(),
        .autoExposureEnabled = isAutoExposureEnabled(),
        .autoWhiteBalanceEnabled = isAutoWhiteBalanceEnabled()
    };
    return stateStruct;
}

// ROS Callbacks
void RosCamClient::stateCallback(const polarimetric_camera_server::CameraStateMessage stateMsg)
{
    if (_stateCallback)
    {
        _lastReceivedState = {
            .gain = stateMsg.gain,
            .exposureUs = stateMsg.exposureUs,
            .frameRate = stateMsg.frameRate,
            .redGain = stateMsg.redGain,
            .greenGain = stateMsg.greenGain,
            .blueGain = stateMsg.blueGain,
            .autoGainEnabled = static_cast<bool>(stateMsg.autoGainEnabled),
            .autoExposureEnabled = static_cast<bool>(stateMsg.autoExposureEnabled),
            .autoWhiteBalanceEnabled = static_cast<bool>(stateMsg.autoWhiteBalanceEnabled)
        };
        _stateCallback->updateStateCallback(_lastReceivedState);
    }
}

void RosCamClient::refreshEvents()
{
    if (!_topicsInitialized)
    {
        ros::NodeHandle* clientNodeHandler = RosHandlerSingleton::getInstance()->getNodeHandler();
        image_transport::ImageTransport* it = RosHandlerSingleton::getInstance()->getImageTransport();

        _camStateSubs = clientNodeHandler->subscribe(_camStateTopicName.c_str(),
            10,
            &RosCamClient::stateCallback,
            (RosCamClient*)this);

        _requestState = clientNodeHandler->advertise<std_msgs::Empty>(
            _camRequestParamsTopicName.c_str(),
            1000);

        _camImagesSubscriber = it->subscribe(_camImgTopicName, 10, &RosCamClient::imagesCallback, this);

        _temperatureSubscriber = clientNodeHandler->subscribe(_camTempTopicName.c_str(),
            10,
            &RosCamClient::temperatureCallback,
            (RosCamClient*)this);
        _topicsInitialized = true;
    }

    {
        std::unique_lock<std::mutex> lck(s_rosInitMtx);
        _isRosInitialized = true;
        s_cvRos.notify_all();
    }

    ros::Rate loop_rate(100);
    while (ros::ok() && _isRunning)
    {
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("ROS refresh thread finished");
}

void RosCamClient::imagesCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr imgPtr;
    try
    {
        imgPtr = cv_bridge::toCvCopy(msg, _usedEncoding);
        {
            std::unique_lock<std::mutex> lck(s_imgMutex);
            imgPtr->image.copyTo(_lastRetrievedImage);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RosCamClient::temperatureCallback(const std_msgs::Float32& msg)
{
    std::unique_lock<std::mutex> lck(s_tempMtx);
    _lastRetrievedTemperature = msg.data;
}
