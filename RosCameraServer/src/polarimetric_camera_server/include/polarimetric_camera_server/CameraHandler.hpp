#ifndef __CAMERA_HANDLER_HPP__
#define __CAMERA_HANDLER_HPP__

// OpenCV includes

// STD includes
#include <string>

// Qt Includes

// Custom includes
#include "PolarimetricCamera/IPolarimetricCamera.hpp"

// ROS includes
#include <image_transport/image_transport.h>
#include "polarimetric_camera_server/CameraParameters.h"
#include "polarimetric_camera_server/ParameterRequested.h"
#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "std_srvs/Empty.h"

/**
 * @brief CameraHandler: This class is a wrapper of all the possible camera drivers.
 *   It helps to handle the case when several cameras are required to be connected at
 *  the same time. During construction, only internal variables are initialized, and
 *  the right driver object is used. The functions available are mainly those of the
 *  IPolarimetricCamera interface.
 *
 *   This class makes available several ROS services and topics to interact with the camera.
 *  Camera parameters can be set and get by services, and the images can be received through topics.
 *  This class allows two operating modes: master and slave. If in master mode, the system
 *  creates images alone, and it generates triggering signals through the back camera connector.
 *  If it is in slave mode, it waits for a trigger signal to take a picture.
*/
class CameraHandler
{
public:
    /**
     * @brief CameraHandler: Class constructor. It will initialize the internal variables,
     *   and it will choose the correct object type for the selected camera.
     *
     * @arg nh: Pointer to the NodeHandler created on the main function.
     * @arg driverSelected: String with a valid driver selected. If the driver name
     *   is not valid, an assertion is thrown.
     * @arg camName: User-defined camera name. This is a way (at least for Basler cameras)
     *   to identify a camera between others. If this string is empty, the first found
     *   camera will be taken. ATTENTION: A camera can only be handled once. If several
     *   objects try to open the same camera, exception from the corresponding libraries
     *   will be thrown.
     * @arg isMaster: If this variable is true, the camera is in MASTER mode, which means
     *   that it does not expects an external triggering signal to take an image, it will
     *   take them at its own pace. At the same time, it will output a triggering signal
     *   from the output connection at the back of the camera. If it is false, the camera
     *   takes images only when the external trigger signal appears.
    */
    CameraHandler(ros::NodeHandle *nh, std::string driverSelected, std::string camName, bool isMaster);
    ~CameraHandler() = default;

    /**
     * @brief initializeTopics: Initialize topics and services to interact with the camera.
     *   It will block until a successfull connection with the camera is produced. If not,
     *  it will try to re-connect every 1 second. Each topic name is prepended by the
     *  user-defined camera name. If this name is an empty string, the topics names
     *  do not have any prepended string. For example, if the topic name is camera_image,
     *  and the camera name is Ace_RGB_Pola_01, then the topic name will be /Ace_RGB_Pola_01/camera_image.
     *  if the camera name is an empty string, the same topic will be called /camera_image.
     *
     * @arg bitDepth: Camera bit-depth to set. For now, only 8 and 12 are
     *  allowed values.
    */
    void initializeTopics(int bitDepth);

    /**
     * @brief getImage: Get an image from the camera synchronously. If the camera
     *  is in MASTER mode, this function will block at maximum 1 exposure time cycle.
     *  If it is in SLAVE mode, this function will block until an external signal
     *  appears + 1 exposure time cycle.
     *
     * @returns Raw captured image in OpenCV format.
    */
    cv::Mat getImage() const {return _camera->getSingleImage();}

    /**
     * @brief getTemperature: Get camera temperature.
     *
     * @return Floating point number with the temperature in degrees celcius.
    */
    float getTemperature() const {return _camera->getTemperature();}

    // \brief stopGrabbing: Clear internal camera variables, detach from it, and put it into idle mode.
    void stopGrabbing() const {_camera->stopGrabbing();}

    /**
     * @brief publishImg: Publish the given image into the camera specific image topic.
     *
     * @arg sensorImgMsg: Image sensor message, already filled with the image to send.
    */
    void publishImg(const sensor_msgs::Image& sensorImgMsg) const {_cameraPub.publish(sensorImgMsg);}

    /**
     * @brief publishTemp: Publish the given float value into the camera specific temperature topic.
     *
     * @arg tempMsg: Float message, already filled with the temperature to send.
    */
    void publishTemp(const std_msgs::Float32& tempMsg) const {_tempPublisher.publish(tempMsg);}

    /**
     * @brief isTemperatureSubscribed: Check if there is any client connected
     *  to the temperature topic.
     *
     * @returns: True if there is at least one client connected. False elsewise.
    */
    bool isTemperatureSubscribed() const {return (_tempPublisher.getNumSubscribers() > 0);}

    /**
     * @brief isImageSubscribed: Check if there is any client connected
     *  to the image topic.
     *
     * @returns: True if there is at least one client connected. False elsewise.
    */
    bool isImageSubscribed() const {return (_cameraPub.getNumSubscribers() > 0);}

private:
    /**
     * @brief changeCameraParameter: Service callback. It will change an specific
     *   parameter from the camera. It will create an struct of the CameraState type,
     *   it will fill it with the newParamMsg parameters, and then it will call
     *   the corresponding CameraDriver function to change the requested parameter.
     *
     * @arg newParamMsg: Message with the ID of the parameter to change (paramToChange field),
     *   and the corresponding value of that parameter. If paramToChange == ALL,
     *   we change all the parameters of the camera that are located in the CameraState struct,
     *   if not, only the corresponding parameter is read, and all the other ones are ignored.
     *
     * @arg res: Empty response message.
     *
     * @return Boolean true if the parameter has been changed successfuly.
    */
    bool changeCameraParameter(polarimetric_camera_server::CameraParameters::Request &newParamMsg,
        polarimetric_camera_server::CameraParameters::Response &res);

    /**
     * @brief isAliveService: Service callback. It will check if the camera is
     *    correctly initialized, and responding.
     *
     * @arg req: Empty request message
     * @arg res: Empty response message
     *
     * @return Boolean true if the camera is alive.
    */
    bool isAliveService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /**
     * @brief getParamService: Service callback. Get certain parameter from the camera.
     *
     * @arg req: Message with the ID of the requested parameter.
     * @arg res: Message with the value of the requested parameter. If req == ALL,
     *    ALL the camera parameters are provided.
     *
     * @return Boolean true the requested parameter has be retrieved successfully.
    */
    bool getParamService(polarimetric_camera_server::ParameterRequested::Request &req,
        polarimetric_camera_server::ParameterRequested::Response &res);

    /**
     * @brief publishCameraState: Read all the camera parameters and publish them
     *    into the correct topic.
    */
    void publishCameraState();

    /**
     * @brief requestCameraParamsCallback: Subscriber callback. When an empty
     *    message is received into this topic, the full camera state is
     *    published into the camera state topic.
     *
     * @arg msg: Empty message.
    */
    void requestCameraParamsCallback(const std_msgs::Empty msg);

    /**
     * @brief changeExposureTime: Update the camera exposure time
     *
     * @arg state: CameraState struct with the new exposure time. Only the
     *  exposure time will be read. All the other parameters will be ignored.
    */
    void changeExposureTime(const CameraState& state);

    /**
     * @brief changeFrameRate: Update the camera frame rate.
     *
     * @arg state: CameraState struct with the new frame rate. Only the
     *  frame rate field will be read. All the other parameters will be ignored.
    */
    void changeFrameRate(const CameraState& state);

    /**
     * @brief changeGain: Update the camera pixels gain.
     *
     * @arg state: CameraState struct with the new gain. Only the
     *  gain field will be read. All the other parameters will be ignored.
    */
    void changeGain(const CameraState& state);

    /**
     * @brief adjustWhiteBalance: Update the camera white balance gains.
     *
     * @arg state: CameraState struct with the new color gains. Only the
     *  red, green and blue gain fields will be read. All the other
     *  parameters will be ignored.
    */
    void adjustWhiteBalance(const CameraState& state);

    /**
     * @brief enableAutoExposure: Change the state of the auto exposure feature.
     *
     * @arg state: CameraState struct with the new auto exposure feature state.
     *  Only the autoExposureEnabled field will be read, and the other fields
     *  will be ignored.
    */
    void enableAutoExposure(const CameraState& state);

    /**
     * @brief enableAutoGain: Change the state of the auto gain feature.
     *
     * @arg state: CameraState struct with the new auto gain feature state.
     *  Only the autoGainEnabled field will be read, and the other fields
     *  will be ignored.
    */
    void enableAutoGain(const CameraState& state);

    /**
     * @brief enableAutoWhiteBalance: Change the state of the auto white balance feature.
     *
     * @arg state: CameraState struct with the new auto white balance feature state.
     *  Only the autoWhiteBalanceEnabled field will be read, and the other fields
     *  will be ignored.
    */
    void enableAutoWhiteBalance(const CameraState& state);

    ros::NodeHandle* _nodeHandler;
    std::string _driverSelected;
    std::string _userCamName;

    /// Camera topics and services names
    std::string _camPubTopicName;
    std::string _camStateTopicName;
    std::string _camTempTopicName;
    std::string _camChangeParameterServiceName;
    std::string _camRequestParamsTopicName;
    std::string _camIsAliveServiceName;
    std::string _requestSpecificParamSrvName;
    std::string _camReqImageServiceName;

    // Camera pointer.
    std::unique_ptr<IPolarimetricCamera> _camera;

    /// Topics publishers and subscribers
    ros::Publisher _statePublisher;
    ros::Publisher _tempPublisher;
    ros::Subscriber _requestParams;

    /// ROS Services
    ros::ServiceServer _changeParamsService;
    ros::ServiceServer _isCameraAliveService;
    ros::ServiceServer _requestSpecificParamService;

    /// Image topic publisher variables
    image_transport::ImageTransport _it;
    image_transport::Publisher _cameraPub;

    bool _isMaster;
};
#endif // __CAMERA_HANDLER_HPP__