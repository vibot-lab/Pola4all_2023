#ifndef __ROS_CAM_CLIENT_HPP__
#define __ROS_CAM_CLIENT_HPP__

// OpenCV includes

// STD includes
#include <map>
#include <memory>
#include <thread>

// Qt Includes

// Pylon includes

// Custom includes
#include <CameraStateCallback.hpp>
#include <PolarimetricCamera/RosCamClient/RosHandlerSingleton.hpp>

// ROS includes
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include <polarimetric_camera_server/CameraStateMessage.h>
#include <std_msgs/Float32.h>

/// Typedef of function definitions that set parameters of the camera
class RosCamClient;
typedef void (RosCamClient::*parameterFunction)(const CameraState&);

/**
 * @brief RosCamClient class
 * This implementation is a client that communicates with a ROS server where the
 * the camera is physically connected. Only 4 topics are used, and the rest is
 * done through services, forcing the synchronization of the connection with the camera.
 * The names of these topics and services are defined in RosNaming.hpp. Until now
 * we count with:
 *  - Topics:
 *      o) Camera image: We subscribe to this topic to receive images from the camera
 *      o) Camera state: We subscribe to this topic to receive the new states
 *          from the camera (either because some auto parameter changed, or
 *          because we requested it).
 *      o) Temperature topic: We subscribe to this topic to know the camera temperature.
 *      o) Request state: We publish into this topic each time we want to receive
 *          the camera state.
 *  - Services
 *      o) Change parameters: We use this service to change the desired parameters
 *      o) Check camera response: Service is used to know if the camera is
 *          responsive or not.
 *      o) Request specific parameter: With this service, we can query no matter
 *          which camera parameter.
*/
class RosCamClient
{
public:
    /**
     * Singleton implementation taken from
     *  https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
    */
    static RosCamClient& getInstance()
    {
        return instance;
    }
    RosCamClient(RosCamClient const&) = delete;
    void operator=(RosCamClient const&)  = delete;

    // Public API
    /**
     * @brief initCam: Initialize ROS topics, and continuous receiving thread.
     *
     * @arg bitDepth: Expected bit depth of the incoming images. Either 8- or 16-bits.
     * @return Boolean true if the initialization was successful.
    */
    bool initCam(int bitDepth = -1);

    /**
     * @brief deinitCam: Shutdown topics, services, and the continuous thread.
    */
    void deinitCam();

    // Public API: Parameters that can be changed
    /**
     * @brief setStateCallback: Change the function to call when the camera state
     *      has been changed.
     *
     * @arg stateCallback: Pointer to a CameraStateCallback class that will be
     *  called when a new state has been set, after a change in any of the
     *  auto features.
    */
    void setStateCallback(CameraStateCallback *stateCallback) { _stateCallback = stateCallback; }

    /**
     * @brief changeExposureTime: Update the camera exposure time
     *
     * @arg state: CameraState struct with the new exposure time. Only the
     *  exposure time will be read. All the other parameters will be ignored.
    */
    void changeExposureTime(const CameraState &state);

    /**
     * @brief changeFrameRate: Update the camera frame rate.
     *
     * @arg state: CameraState struct with the new frame rate. Only the
     *  frame rate field will be read. All the other parameters will be ignored.
    */
    void changeFrameRate(const CameraState &state);

    /**
     * @brief changeGain: Update the camera pixels gain.
     *
     * @arg state: CameraState struct with the new gain. Only the
     *  gain field will be read. All the other parameters will be ignored.
    */
    void changeGain(const CameraState &state);

    /**
     * @brief adjustWhiteBalance: Update the camera white balance gains.
     *
     * @arg state: CameraState struct with the new color gains. Only the
     *  red, green and blue gain fields will be read. All the other
     *  parameters will be ignored.
    */
    void adjustWhiteBalance(const CameraState &state);

    // Accessors
    /**
     * @brief getRawImage: Get a raw capture from the camera asynchronously.
     *  The returned object is the image holded in an internal buffer that is
     *  updated each time the image topic receives a new image.
     *
     * @arg outputImages [output] OpenCV image. If the image cannot be
     *  retrieved properly an empty image is returned.
    */
    void getRawImage(cv::Mat& img) const;

    /**
     * @brief getSingleImageSynchronously: Get a raw capture from the camera
     *  synchronously. This function will block until a new frame from the camera
     *  is received.
     *
     * @arg outputImages [output] OpenCV image. If the image cannot be
     *  retrieved properly an empty image will be returned.
    */
    void getSingleImageSynchronously(cv::Mat& img) const;

    /**
     * @brief getGainLimits: Get the minimum and maximum allowed values for the pixel gain.
     *
     * @return std::vector with two elements. The first one is the minimum, and
     *      the second one is the maximum allowed gain value, expressed in dB.
    */
    std::vector<int> getGainLimits() const;

    /**
     * @brief getExposureLimits: Get the minimum and maximum allowed values for the exposure time.
     *
     * @return std::vector with two elements. The first one is the minimum, and
     *      the second one is the maximum allowed exposure time value expressed
     *      in micro seconds.
    */
    std::vector<double> getExposureLimits() const;

    /**
     * @brief getTemperature: Get the camera temperature.
     *
     * @returns float with the actual device temperature.
    */
    float getTemperature() const;

    /**
     * @brief getFrameRate: Get the actual camera frame rate.
     *
     * @return Integer that represents the frame rate in fps.
    */
    int getFrameRate() const;

    /**
     * @brief getExposureTime: Get the current camera exposure time.
     *
     * @return Float value with the current exposure time expressed in uS.
    */
    double getExposureTime() const;

    /**
     * @brief getGain: Get the current pixel gain.
     *
     * @return Double with the current pixel gain, expressed in dB.
    */
    double getGain() const;

    /**
     * @brief getWhiteBalanceGains: Get the camera color gains separately.
     *
     * @return std::vector with 3 values. The first one represents the red
     *      relative gain of the red with respect the other two colors.
     *      The second one is the relative gain of the green with respect to
     *      the other two colors. The thrid one is the relative gain of the blue
     *      with respect to the other two colors.
    */
    std::vector<double> getWhiteBalanceGains() const;

    /**
     * @brief enableAutoExposure: Change the state of the auto exposure feature.
     *
     * @arg state: CameraState struct with the new auto exposure feature state.
     *  Only the autoExposureEnabled field will be read, and the other fields
     *  will be ignored.
    */
    void enableAutoExposure(const CameraState &state);

    /**
     * @brief enableAutoGain: Change the state of the auto gain feature.
     *
     * @arg state: CameraState struct with the new auto gain feature state.
     *  Only the autoGainEnabled field will be read, and the other fields
     *  will be ignored.
    */
    void enableAutoGain(const CameraState &state);

    /**
     * @brief enableAutoWhiteBalance: Change the state of the auto white balance feature.
     *
     * @arg state: CameraState struct with the new auto white balance feature state.
     *  Only the autoWhiteBalanceEnabled field will be read, and the other fields
     *  will be ignored.
    */
    void enableAutoWhiteBalance(const CameraState &state);

    // Checking camera state
    /**
     * @brief isAutoGainEnabled: Check the state of the auto gain feature.
     *
     * @return Boolean. True if the auto gain is enabled.
    */
    bool isAutoGainEnabled() const;

    /**
     * @brief isAutoExposureEnabled: Check the state of the auto exposure time feature.
     *
     * @return Boolean. True if the auto exposure time is enabled.
    */
    bool isAutoExposureEnabled() const;

    /**
     * @brief isAutoWhiteBalanceEnabled: Check the state of the auto white balance feature.
     *
     * @return Boolean. True if the auto white balance is enabled.
    */
    bool isAutoWhiteBalanceEnabled() const;

    /**
     * @brief isAlive: Check if the camera is connected and responding.
     *
     * @return Boolean. True if the camera is correctly connected and
     *      we can communicate with it.
    */
    bool isAlive() const;

    /**
     * @brief changeCameraParameters: Update one or all the camera parameters.
     *
     * @arg paramToChange: Option from the CameraParams. If this argument is
     *  ALL, all the parameters from the state will be updated. In any other case,
     *  only the specified parameter will be updated.
     *
     * @arg state: CameraState struct with the new values of the parameter/s to
     *  be updated.
    */
    void changeCameraParameters(CameraParams paramToChange, CameraState state);

    /**
     * @brief Get a full camera state struct.
    */
    CameraState getCameraState() const;

private:
    /**
     * @brief Constructor. It will only set the initial values of the internal variables.
    */
    RosCamClient();
    ~RosCamClient();

    /**
     * @brief initializeRosClient: Initialize the ROS node and topics (Subscribers
     *      and publishers). This function also waits for the first subscriber
     *      to the topics to confirm that the client is ready to work.
     *
     * @return Boolean. True if the initialization was successful.
    */
    bool initializeRosClient();

    /**
     * @brief stateCallback: Subscriber callback that is executed when a message
     *    from the state topic is received.
     *
     * @arg stateMsg: Received message from the state topic.
    */
    void stateCallback(const polarimetric_camera_server::CameraStateMessage stateMsg);

    /**
     * @brief imagesCallback: Subscriber callback executed when a message from the image
     *      topic is received.
     *
     * @arg msg: Received message from the camera image.
    */
    void imagesCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief temperatureCallback: Subscriber callback executed when a message from the temperature
     *      topic is received.
     *
     * @arg stateMsg: Received message from the camera temperature.
    */
    void temperatureCallback(const std_msgs::Float32& msg);

    /**
     * @brief callChangeParamsService: Function that changes parameters in the camera,
     *      through the corresponding service.
     *
     * @arg params: Struct with the parameters values to be changed.
     * @arg paramToChange: Enum that identifies which parameter the user wants to change.
    */
    void callChangeParamsService(const CameraState &params, const CameraParams& paramToChange);

    /**
     * @brief requestParam: Read a camera parameter by using ROS services.
     *
     * @arg params [output]: Reference to the CameraState struct in which the retrieved
     *   parameter will be stored.
     *
     * @arg requestedParam: Enum value that tells which parameter wants to be requested.
    */
    bool requestParam(CameraState &params, const CameraParams& requestedParam) const;

    /**
     * @brief emitState: Send a message to request the state of the camera. This
     *  function does not return the state, but the corresponding callback will be
     *  called as soon as the state arrives and this will be communicated to the
     *  upper layer.
    */
    void emitState();

    /**
     * @brief refreshEvents: Function that refreshes the required events of
     *  ROS
    */
    void refreshEvents();

    /// External function to be called whenever the ROS client wants to communicate the camera state.
    CameraStateCallback *_stateCallback;

    /// Topics names variables
    std::string _camImgTopicName;
    std::string _camStateTopicName;
    std::string _camTempTopicName;
    std::string _camChangeParameterServiceName;
    std::string _camRequestParamsTopicName;
    std::string _camIsAliveServiceName;
    std::string _requestSpecificParamSrvName;

    /// Subscribers and publishers members
    ros::Subscriber _camStateSubs;
    image_transport::Subscriber _camImagesSubscriber;
    ros::Subscriber _temperatureSubscriber;
    ros::Publisher _requestState;

    /// Internal variables
    CameraState _lastReceivedState;
    ros::Duration _topicWaitTimeout;
    cv::Mat _lastRetrievedImage;
    float _lastRetrievedTemperature;

    static RosCamClient instance;

    bool _isRunning;
    bool _isRosInitialized;
    std::thread _freeRunThread;
    bool _topicsInitialized;

    std::map<CameraParams, parameterFunction> _parametersCallbacks;
    std::string _usedEncoding;
};

#endif // __ROS_CAM_CLIENT_HPP__