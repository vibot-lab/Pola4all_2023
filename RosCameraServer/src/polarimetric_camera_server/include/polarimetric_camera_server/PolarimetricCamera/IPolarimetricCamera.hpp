#ifndef __IPOLARIMETRIC_CAMERA_HPP__
#define __IPOLARIMETRIC_CAMERA_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// STD includes
#include <iostream>
#include <map>
#include <unistd.h>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes
#include <CameraTypes.hpp>

/// Typedef of function definitions that set parameters of the camera
class IPolarimetricCamera;
typedef void (IPolarimetricCamera::*parameterFunction)(const CameraState&);

/**
 * @brief IPolarimetricCamera class
 * This class constitues an interface to all the camera connection types.
 * It provides some virtual functions that are specific of how the connection
 * with the camera is done, and some other method for image processing.
 *
 * This interface creates an abstraction layer that allow us to create several
 * types of connections, and to integrate them easily into the project. Since
 * several methods cannot be implemented here, they have been declared as pure
 * virtual.
 *
 * This interface allows to:
 * Get / set:
 *  - Frame rate
 *  - Exposure time
 *  - Pixel gain
 *  - White balance
 *  - Start / stop camera free run mode
 *
 * Enable / disable auto features:
 *  - Auto Exposure
 *  - Auto gain
 *  - Auto white balance
 *
 * Get:
 *  - Raw image
 *  - Connection name
 *  - Pixel gain limits
 *  - Exposure limits
 *  - Device temperature
 *
 *
 * Misc:
 *  - It provides a function that will be called each time the camera wants to
 *      notify an new state, when changing the state of an Auto feature.
*/
class IPolarimetricCamera
{
public:
    /**
     * @brief Constructor
    */
    IPolarimetricCamera()
    {
        _parametersCallbacks[AUTO_EXPOSURE] = &IPolarimetricCamera::enableAutoExposure;
        _parametersCallbacks[AUTO_GAIN] = &IPolarimetricCamera::enableAutoGain;
        _parametersCallbacks[AUTO_WHITE_BALANCE] = &IPolarimetricCamera::enableAutoWhiteBalance;
        _parametersCallbacks[FRAME_RATE] = &IPolarimetricCamera::changeFrameRate;
        _parametersCallbacks[EXPOSURE_TIME] = &IPolarimetricCamera::changeExposureTime;
        _parametersCallbacks[GAIN] = &IPolarimetricCamera::changeGain;
        _parametersCallbacks[WHITE_BALANCE_GAINS] = &IPolarimetricCamera::adjustWhiteBalance;
    }
    virtual ~IPolarimetricCamera() = default;

    // Public API: Start / Stop free-run
    /**
     * @brief startGrabbing: Put the camera into continuous mode.
     *  Several operations are done here. If the camera is connected and activated,
     *  but not in free mode, the function will try to put the camera into
     * free-run mode. If the camera is not activated, then the USB connection
     * is done, and then the camera parameters initialization is done to ensure
     * that the camera has a proper initial state.
     *
     * @arg bitDepth: Integer value indicating the pixel resolution to use(8,
     *  10, 12, etc).
     * @arg camToConnect: String with the camera user-defined name. If empty, the
     *   first found camera will be used.
     * @return Boolean true if it was possible to put the camera into free-run mode.
    */
    virtual bool startGrabbing(int bitDepth, std::string camToConnect = "") = 0;

    /**
     * @brief stopGrabbing: Stop the camera free-run mode, and release the camera
     * so other programs can take the control of it, if they want to.
    */
    virtual void stopGrabbing() = 0;

    /**
     * @brief initializeTriggeringMode: Set the camera in external trigger mode.
     *
     * @arg bitDepth: Integer value indicating the pixel resolution to use(8,
     *  10, 12, etc).
     * @arg camToConnect: String with the camera user-defined name. If empty, the
     *   first found camera will be used.
     * @arg returns boolean true if the initialization have been carried out correctly.
    */
    virtual bool initializeTriggeringMode(int bitDepth, std::string camToConnect = "") = 0;

    /// Getters
    /**
     * @brief getSingleImage: Get a raw capture from the camera asynchronously.
     *  The returned object is the image holded in an internal buffer that is
     *  updated in a secondary thread that is continuously reading the camera.
     *
     * @arg outputImages [output] Retrieved image as an OpenCV image. If the image cannot be
     *  retrieved properly an empty image will be returned.
    */
    virtual cv::Mat getSingleImage() = 0;

    /**
     * @brief getGainLimits: Get the minimum and maximum allowed values for the pixel gain.
     *
     * @return std::vector with two elements. The first one is the minimum, and
     *      the second one is the maximum allowed gain value, expressed in dB.
    */
    virtual std::vector<int> getGainLimits() const = 0 ;

    /**
     * @brief getExposureLimits: Get the minimum and maximum allowed values for the exposure time.
     *
     * @return std::vector with two elements. The first one is the minimum, and
     *      the second one is the maximum allowed exposure time value expressed
     *      in micro seconds.
    */
    virtual std::vector<double> getExposureLimits() const = 0;

    /**
     * @brief getTemperature: Get the camera temperature.
     *
     * @returns float with the actual device temperature.
    */
    virtual float getTemperature() const = 0;

    /**
     * @brief isAlive: Check if the camera is connected and responding.
     *
     * @return Boolean. True if the camera is correctly connected and
     *      we can communicate with it.
    */
    virtual bool isAlive() const = 0;

    /**
     * @brief getFrameRate: Get the actual camera frame rate.
     *
     * @return Integer that represents the frame rate in fps.
    */
    virtual int getFrameRate() const = 0;

    /**
     * @brief getExposureTime: Get the current camera exposure time.
     *
     * @return Float value with the current exposure time expressed in uS.
    */
    virtual double getExposureTime() const = 0;

    /**
     * @brief getGain: Get the current pixel gain.
     *
     * @return Double with the current pixel gain, expressed in dB.
    */
    virtual double getGain() const = 0;

    /**
     * @brief getWhiteBalanceGains: Get the camera color gains separately.
     *
     * @return std::vector with 3 values. The first one represents the red
     *      relative gain of the red with respect the other two colors.
     *      The second one is the relative gain of the green with respect to
     *      the other two colors. The thrid one is the relative gain of the blue
     *      with respect to the other two colors.
    */
    virtual std::vector<double> getWhiteBalanceGains() const = 0;

    /**
     * @brief isAutoGainEnabled: Check the state of the auto gain feature.
     *
     * @return Boolean. True if the auto gain is enabled.
    */
    virtual bool isAutoGainEnabled() const = 0;

    /**
     * @brief isAutoExposureEnabled: Check the state of the auto exposure time feature.
     *
     * @return Boolean. True if the auto exposure time is enabled.
    */
    virtual bool isAutoExposureEnabled() const = 0;

    /**
     * @brief isAutoWhiteBalanceEnabled: Check the state of the auto white balance feature.
     *
     * @return Boolean. True if the auto white balance is enabled.
    */
    virtual bool isAutoWhiteBalanceEnabled() const = 0;

    /**
     * @brief changeCameraParameters: Update one or all the camera parameters.
     *
     * @arg paramToChange: Option from the CameraParams. If this argument is
     *  ALL, all the parameters from the state will be updated. In any other case,
     *  only the specified parameter will be updated.
     *
     * @arg state: CameraState struct with the new values of the parameters to
     *  be updated.
    */
    void changeCameraParameters(CameraParams paramToChange, CameraState state)
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

    /**
     * @brief Get a full camera state struct.
    */
    CameraState getCameraState() const
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

private:
    /**
     * @brief changeExposureTime: Update the camera exposure time
     *
     * @arg state: CameraState struct with the new exposure time. Only the
     *  exposure time will be read. All the other parameters will be ignored.
    */
    virtual void changeExposureTime(const CameraState& state) = 0;

    /**
     * @brief changeFrameRate: Update the camera frame rate.
     *
     * @arg state: CameraState struct with the new frame rate. Only the
     *  frame rate field will be read. All the other parameters will be ignored.
    */
    virtual void changeFrameRate(const CameraState& state) = 0;

    /**
     * @brief changeGain: Update the camera pixels gain.
     *
     * @arg state: CameraState struct with the new gain. Only the
     *  gain field will be read. All the other parameters will be ignored.
    */
    virtual void changeGain(const CameraState& state) = 0;

    /**
     * @brief adjustWhiteBalance: Update the camera white balance gains.
     *
     * @arg state: CameraState struct with the new color gains. Only the
     *  red, green and blue gain fields will be read. All the other
     *  parameters will be ignored.
    */
    virtual void adjustWhiteBalance(const CameraState& state) = 0;

    /**
     * @brief enableAutoExposure: Change the state of the auto exposure feature.
     *
     * @arg state: CameraState struct with the new auto exposure feature state.
     *  Only the autoExposureEnabled field will be read, and the other fields
     *  will be ignored.
    */
    virtual void enableAutoExposure(const CameraState& state) = 0;

    /**
     * @brief enableAutoGain: Change the state of the auto gain feature.
     *
     * @arg state: CameraState struct with the new auto gain feature state.
     *  Only the autoGainEnabled field will be read, and the other fields
     *  will be ignored.
    */
    virtual void enableAutoGain(const CameraState& state) = 0;

    /**
     * @brief enableAutoWhiteBalance: Change the state of the auto white balance feature.
     *
     * @arg state: CameraState struct with the new auto white balance feature state.
     *  Only the autoWhiteBalanceEnabled field will be read, and the other fields
     *  will be ignored.
    */
    virtual void enableAutoWhiteBalance(const CameraState& state) = 0;

    std::map<CameraParams, parameterFunction> _parametersCallbacks;
};

#endif //__IPOLARIMETRIC_CAMERA_HPP__