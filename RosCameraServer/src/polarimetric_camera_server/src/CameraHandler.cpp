// OpenCV includes

// STD includes

// Qt Includes

// Custom includes
#include "CameraHandler.hpp"
#include "PolarimetricCamera/BaslerCamDriver/BaslerCamDriver.hpp"
#include "PolarimetricCamera/TemplateDriver/TemplateDriver.hpp"

// ROS includes
#include "polarimetric_camera_server/CameraStateMessage.h"
#include "RosNaming.hpp"

CameraHandler::CameraHandler(ros::NodeHandle *nh, std::string driverSelected, std::string camName, bool isMaster) :
    _nodeHandler(nh),
    _driverSelected(driverSelected),
    _userCamName(camName),
    _camPubTopicName(camName + "/" + CAMERA_IMAGE_TOPIC_NAME),
    _camStateTopicName(camName + "/" + CAMERA_STATE_TOPIC_NAME),
    _camTempTopicName(camName + "/" + CAMERA_TEMPERATURE_TOPIC_NAME),
    _camChangeParameterServiceName(camName + "/" + CAMERA_CHANGE_PARAM_SERVICE),
    _camRequestParamsTopicName(camName + "/" + CAMERA_REQUEST_PARAMS_TOPIC_NAME),
    _camIsAliveServiceName(camName + "/" + CAMERA_IS_ALIVE_SERVICE),
    _requestSpecificParamSrvName(camName + "/" + CAMERA_REQUEST_SPECIFIC_PARAM_SERVICE),
    _it(*nh),
    _isMaster(isMaster)
{
    if (_driverSelected == "BaslerUSB")
    {
        _camera = std::unique_ptr<IPolarimetricCamera>(new BaslerCamDriver());
    }
    else if (_driverSelected == "TemplateDriver")
    {
        _camera = std::unique_ptr<IPolarimetricCamera>(new TemplateClient());
    }
    else if (_driverSelected == "EthernetDriver")
    {
        ROS_WARN("Ethernet driver not implemented yet!");
        assert(0);
    }
    else
    {
        ROS_WARN("Camera driver not implemented!!");
        assert(0);
    }
}

void CameraHandler::initializeTopics(int camBitDepth)
{
    _cameraPub = _it.advertise(_camPubTopicName.c_str(), 1);

    _statePublisher = _nodeHandler->advertise<polarimetric_camera_server::CameraStateMessage>(
        _camStateTopicName.c_str(),
        1);

    _tempPublisher = _nodeHandler->advertise<std_msgs::Float32>(
        _camTempTopicName.c_str(),
        1);

    _requestParams = _nodeHandler->subscribe(_camRequestParamsTopicName.c_str(),
        1,
        &CameraHandler::requestCameraParamsCallback,
        (CameraHandler*)this);

    _changeParamsService = _nodeHandler->advertiseService(
        _camChangeParameterServiceName.c_str(),
        &CameraHandler::changeCameraParameter,
        (CameraHandler*)this);

    _isCameraAliveService = _nodeHandler->advertiseService(
        _camIsAliveServiceName.c_str(),
        &CameraHandler::isAliveService,
        (CameraHandler*)this);

    _requestSpecificParamService = _nodeHandler->advertiseService(
        _requestSpecificParamSrvName.c_str(),
        &CameraHandler::getParamService,
        (CameraHandler*)this);

    bool isCameraConnected = false;
    ROS_INFO("Connecting to camera %s", _userCamName.c_str());
    ros::Rate waitRate(1);
    while (ros::ok() && !isCameraConnected)
    {
        if (_isMaster)
        {
            isCameraConnected = _camera->startGrabbing(camBitDepth, _userCamName);
        }
        else
        {
            isCameraConnected = _camera->initializeTriggeringMode(camBitDepth, _userCamName);
        }
        if (!isCameraConnected)
        {
            waitRate.sleep();
        }
    }

    if (isCameraConnected)
    {
        ROS_INFO("Connected to camera %s", _userCamName.c_str());
    }
}

bool CameraHandler::getParamService(
    polarimetric_camera_server::ParameterRequested::Request &req,
    polarimetric_camera_server::ParameterRequested::Response &res)
{
    CameraParams requestedParam = static_cast<CameraParams>(req.paramRequested);
    switch(requestedParam)
    {
        case ALL:
        {
            std::vector<double> gains = _camera->getWhiteBalanceGains();
            res.gain = _camera->getGain();
            res.exposureUs = _camera->getExposureTime();
            res.frameRate = _camera->getFrameRate();
            res.redGain = gains[0];
            res.greenGain = gains[1];
            res.blueGain = gains[2];
            res.autoGainEnabled = _camera->isAutoGainEnabled();
            res.autoExposureEnabled = _camera->isAutoExposureEnabled();
            res.autoWhiteBalanceEnabled = _camera->isAutoWhiteBalanceEnabled();
            break;
        }
        case AUTO_EXPOSURE:
        {
            res.autoExposureEnabled = _camera->isAutoExposureEnabled();
            break;
        }
        case AUTO_GAIN:
        {
            res.autoGainEnabled = _camera->isAutoGainEnabled();
            break;
        }
        case AUTO_WHITE_BALANCE:
        {
            res.autoWhiteBalanceEnabled = _camera->isAutoWhiteBalanceEnabled();
            break;
        }
        case FRAME_RATE:
        {
            res.frameRate = _camera->getFrameRate();
            break;
        }
        case EXPOSURE_TIME:
        {
            res.exposureUs = _camera->getExposureTime();
            break;
        }
        case GAIN:
        {
            res.gain = _camera->getGain();
            break;
        }
        case WHITE_BALANCE_GAINS:
        {
            std::vector<double> gains = _camera->getWhiteBalanceGains();
            res.redGain = gains[0];
            res.greenGain = gains[1];
            res.blueGain = gains[2];
            break;
        }
        default:
        {
            ROS_FATAL("Parameter requested not available. Did you updated the CameraTypes in the client side and not in the RosServer ?");
            return false;
        }
    }
    return true;
}

bool CameraHandler::changeCameraParameter(
    polarimetric_camera_server::CameraParameters::Request &newParamMsg,
    polarimetric_camera_server::CameraParameters::Response &res)
{
    CameraParams paramToChange = static_cast<CameraParams>(newParamMsg.paramToChange);
    // IMPORTANT: If you add more types to the CameraParams class, be sure
    // to add the corresponding case here
    assert(NUM_OF_PARAMS == 8);

    if (paramToChange > NUM_OF_PARAMS)
    {
        ROS_FATAL("The parameter to change does not match with the one "
            "specified in the CameraStateMessage.msg file. "
            "Did you update the CameraTypes.hpp and not the "
            "CameraStateMessage.msg file ?");
        assert(0);
    }

    CameraState state = {
        .gain = newParamMsg.gain,
        .exposureUs = newParamMsg.exposureUs,
        .frameRate = newParamMsg.frameRate,
        .redGain = newParamMsg.redGain,
        .greenGain = newParamMsg.greenGain,
        .blueGain = newParamMsg.blueGain,
        .autoGainEnabled = static_cast<bool>(newParamMsg.autoGainEnabled),
        .autoExposureEnabled = static_cast<bool>(newParamMsg.autoExposureEnabled),
        .autoWhiteBalanceEnabled = static_cast<bool>(newParamMsg.autoWhiteBalanceEnabled)
    };
    _camera->changeCameraParameters(paramToChange, state);
    return true;
}

bool CameraHandler::isAliveService(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
{
    (void)req;
    (void)res;
    return _camera->isAlive();
}

void CameraHandler::publishCameraState()
{
    ROS_INFO("CameraHandler: Seding camera state");
    polarimetric_camera_server::CameraStateMessage state;
    std::vector<double> gains = _camera->getWhiteBalanceGains();
    state.gain = _camera->getGain();
    state.exposureUs = _camera->getExposureTime();
    state.frameRate = _camera->getFrameRate();
    state.redGain = gains[0];
    state.greenGain = gains[1];
    state.blueGain = gains[2];
    state.autoGainEnabled = _camera->isAutoGainEnabled();
    state.autoExposureEnabled = _camera->isAutoExposureEnabled();
    state.autoWhiteBalanceEnabled = _camera->isAutoWhiteBalanceEnabled();
    _statePublisher.publish(state);
}

void CameraHandler::requestCameraParamsCallback(const std_msgs::Empty msg)
{
    (void)msg;
    publishCameraState();
}

void CameraHandler::changeExposureTime(const CameraState& state)
{
    _camera->changeCameraParameters(EXPOSURE_TIME, state);
}

void CameraHandler::changeFrameRate(const CameraState& state)
{
    _camera->changeCameraParameters(FRAME_RATE, state);
}

void CameraHandler::changeGain(const CameraState& state)
{
    _camera->changeCameraParameters(GAIN, state);
}

void CameraHandler::adjustWhiteBalance(const CameraState& state)
{
    _camera->changeCameraParameters(WHITE_BALANCE_GAINS, state);
}

void CameraHandler::enableAutoExposure(const CameraState& state)
{
    _camera->changeCameraParameters(AUTO_EXPOSURE, state);
}

void CameraHandler::enableAutoGain(const CameraState& state)
{
    _camera->changeCameraParameters(AUTO_GAIN, state);
}

void CameraHandler::enableAutoWhiteBalance(const CameraState& state)
{
    _camera->changeCameraParameters(AUTO_WHITE_BALANCE, state);
}
