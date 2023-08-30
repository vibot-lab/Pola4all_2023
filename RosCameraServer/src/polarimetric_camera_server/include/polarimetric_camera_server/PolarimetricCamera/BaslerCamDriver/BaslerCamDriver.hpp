#ifndef __CAMERA_DRIVER_HPP__
#define __CAMERA_DRIVER_HPP__

// OpenCV includes
#include <opencv2/core/core.hpp>

// STD includes
#include <memory>

// Qt Includes

// Pylon includes
#include <pylon/PylonIncludes.h>

// Custom includes
#include "CameraTypes.hpp"
#include "PolarimetricCamera/IPolarimetricCamera.hpp"

/**
 * @brief BaslerCamDriver class. This is the most basic camera driver we can have.
 * It uses the Pylon SDK in order to talk to the camera. Since the camera is
 * a USB3 device, then this driver works for the camera connected to the USB
 * of the machine where this code runs.
 *
 * This class implements a wrapper of the Pylon SDK to ease the usage
 * of the polarimetric camera Basler ace acA2440-75u. This driver works also for the
 * color version, the monochrome version, and the polarimetric versions of the camera.
 * No image processing is done in this wrapper, only camera communication functions.
 *
 * Between the implemented functions, we have the start and stop the continuous mode,
 * change the frame rate, change the exposure time, change the pixel gain, and
 * enable or disable auto features, like the auto exposure, auto gain and auto
 * white balance. Also, we have a function that retrieves a raw image from the camera,
 * and to initialize the camera in triggering mode.
 *
 * For the RGB polarization version, the camera provides a polarized,
 * colored image in this disposition (Sony IMX250MYR sensor):
 *
 *                  || R_135 | R_0  || G_135 | G_0  ||
 *                  || R_90  | R_45 || G_90  | G_45 ||
 *                  || G_135 | G_0  || B_135 | B_0  ||
 *                  || G_90  | G_45 || B_90  | B_45 ||
 *
 * where G_ indicates the pixels with green color filter, B_ indicates the pixels
 * with blue color filter, and R_ indicates the pixels with red color filter.
 * 0, 45, 90, and 135 are the polarization angles of each linear filter placed over the
 * pixel. This pattern is present in all the raw image.
 *
 * NOTE 1: The camera has several operation modes, but since the firmware present
 * in the camera acA2440-75ucPOL is the same as its corresponding color camera
 * acA2440-75uc, some features are not useful, since they consider the pixel
 * matrix as being only color, ignoring that actually, it is a polarized, colored
 * pixel matrix. Between all the operation modes: RGB, BGR, Mono8, Mono12,
 * BayerRG8, BayerRG12, only the BayerRG8 and BayerRG12 are useful.
 * These operation modes operation mode returns you a MONOCHROME IMAGE (single channel).
 * MonoX will take the BayerRGX image, and compute the luminiscence at each Bayer pattern.
 * RGB and BGR produces the Bayer interpolation in the middle, which in
 * our case, is not useful neither.
 * The most raw image I we can take is the BayerRG8 (for 8-bits resolution)
 * or BayerRG12 (for 12-bits resolution), and then do image processing by software.
 *
 * NOTE 2: The White balance feature is not useful. The reason for it is the same
 * as in NOTE 1: The software thinks it has a color image only, and not a polarized,
 * colored image. The color camera has this pixel disposition:
 *
 *                              || R | G ||
 *                              || G | B ||
 *
 * As a consequence, the red gain of the white balance only
 * affects the pixels where the polarization angle is 135 degrees, the green gain
 * affects the polarizations at 90 and 0, and the blue one, the pixels with
 * polarization angle of 45 degrees. For this reason, the auto white balance feature
 * should not be used either.
*/
class BaslerCamDriver : public IPolarimetricCamera
{
public:
    /**
     * @brief class Constructor
    */
    BaslerCamDriver();
    ~BaslerCamDriver();

    // Public API: Getting images related functions
    bool startGrabbing(int bitDepth, std::string camToConnect = "") override;
    void stopGrabbing() override;
    bool initializeTriggeringMode(int bitDepth, std::string camToConnect = "") override;

    /**
     * @brief getSingleImage: Get a raw capture from the camera. If the camera is
     *  not properly initialized, and placed in free-run mode, this function
     *  try to do the initialization once. This is a blocking function. It will
     *  return when an image is ready, or when it reaches the camera acquisition timeout
     *  (8 seconds).
     *
     * @return Retrieved image as an OpenCV image. If the image cannot be retrieved properly
     * (the camera is not responding for instance), an empty image is returned.
    */
    cv::Mat getSingleImage() override;

    // Public API: Parameters that can be changed
    /**
     * @brief changeExposureTime: Change the camera exposure time. If the camera
     *  is not connected, an exception is thrown. Also, the given value must
     *  between the camera limits (the camera has a minimum and maximum exposure
     *  time). Since the frame rate is related with the time the camera exposure time,
     *  when we change the exposure time, the frame rate might be changed
     *  automatically by the camera.
     *
     * @arg exposure_us: New exposure time, expressed as in micro seconds.
    */
    void changeExposureTime(const CameraState& state) override;

    /**
     * @brief changeFrameRate: Change the frame rate. It must be a positive value
     *  (an assertion is thrown if this value is negative). In order to execute
     *  this operation, the camera must be connected and opened.
     *  Since the frame rate is related with the camera exposure time,
     *  the frame rate change might be ignored.
     *
     * @arg new_frame_rate: New frame rate to be set.
    */
    void changeFrameRate(const CameraState& state) override;

    /**
     * @brief changeGain: Change the pixel gain. All measurements from
     *  the pixels in the image will be amplified by the especified value in dB.
     *  This gain can be analog or digital, dependending on the expressed value.
     *  In this model, if the new_gain > 24 dB, the gain is digital, with maximum
     *  value of 32 dB. Between 0 and 24 dB, the gain is analog. We decided
     *  to use only analog gains, so the maximum value we can provide for the
     *  gain is 24, and the minimum is 0.
     *
     * @arg new_gain: New gain to set for the pixels, in dB.
    */
    void changeGain(const CameraState& state) override;

    /**
     * @brief adjustWhiteBalance: Change the white balance gains. The gains are
     *  relative to the other colors (Red, Green and Blue). The minimum value for
     *  each gain is 1 (assert). This feature is inherited from the color cameras,
     *  and it has not the desired effect on the polarization camera. See NOTE 2
     *  in the class explanation. In order to execute this operation, the
     *  camera must be connected and opened.
     *
     * @arg redGain: Relative gain of the red with regard the green and the blue.
     * @arg greeGain: Relative gain of the green with regard the red and the blue.
     * @arg blueGain: Relative gain of the blue with regard the red and the green.
    */
    void adjustWhiteBalance(const CameraState& state) override;

    /**
     * @brief enableAutoExposure: Change the camera to or from auto exposure mode.
     *  In order to execute this operation, the camera must be connected and opened.
     *
     * @arg enabled: Boolean. If true, the auto exposure mode is enabled.
    */
    void enableAutoExposure(const CameraState& state) override;

    /**
     * @brief enableAutoGain: Change the camera to or from auto gain mode.
     *  In order to execute this operation, the camera must be connected and opened.
     *
     * @arg enabled: Boolean. If true, the auto gain mode is enabled.
    */
    void enableAutoGain(const CameraState& state) override;

    /**
     * @brief enableAutoWhiteBalance: Change the camera to or from auto white balance mode.
     *  In order to execute this operation, the camera must be connected and opened.
     *  As explained in NOTE 2 of the class explanation, this mode does not work
     *  as expected.
     *
     * @arg enabled: Boolean. If true, the auto white balance mode is enabled.
    */
    void enableAutoWhiteBalance(const CameraState& state) override;

    // Accessors
    /**
     * @brief getGainLimits: Get the limits of the we can set in the camera.
     *  Since these values has been set from the data sheet of the camera,
     *  we do not need to connect to the camera in order to get them.
     *
     * @return std::vector with two integers. The first element is the minimum
     *  gain value, and the second element is the maximum gain value.
    */
    std::vector<int> getGainLimits() const {return _gainLimits;}

    /**
     * @brief getGainLimits: Get the limits of the we can set in the camera.
     *  In order to request these values, we need to have called the initializeCamera
     *  at least once.
     *
     * @return std::vector with two double. The first element is the minimum
     *  gain value, and the second element is the maximum gain value. If the camera
     *  has not been connected before, it returns an empty vector.
    */
    std::vector<double> getExposureLimits() const{return _exposureLimits;}

    /**
     * @brief getTemperature: Retrieve the device temperature. The camera must
     *  be connected and opened in order to request this parameter.
     *
     * @return A float value with the retrieved temperature in degrees Celcius.
     *  If the camera is not initialized, this function returns 0.
    */
    float getTemperature() const;

    /**
     * @brief isAlive: Check if the camera is connected and opened correctly.
     *
     * @return Boolean. It returns true if the camera is correctly connected and
     *  opened.
    */
    bool isAlive() const;

    /**
     * @brief getFrameRate: Request the current camera frame rate. If the
     *  camera is not connected, this value is zero.
    */
    int getFrameRate() const;

    /**
     * @brief getExposureTime: Get the camera exposure time.
     *
     * @return Double with the exposure time measured in micro seconds.
    */
    double getExposureTime() const;

    /**
     * @brief getGain: Retrieve the pixel gain. The camera must be
     *  connected and opened in order to retrieve these values.
     *
     * @return Double with the pixel gain expressed in dB. If the camera is not
     *  connected, it returns 0.
    */
    double getGain() const;

    /**
     * @brief getWhiteBalanceGains: Get the color gains of the white balance.
     *  The camera must be connected and opened in order to retrieve these values.
     *
     * @return std::vector of doubles with 3 elements: red gain, green gain,
     *  and blue gain. If the camera is not connected, it returns an empty vector.
    */
    std::vector<double> getWhiteBalanceGains() const;

    /**
     * @brief isAutoGainEnabled: Check if the camera auto gain feature is enabled.
     *
     * @return Boolean. It returns true if the auto gain feature is enabled.
    */
    bool isAutoGainEnabled() const {return _isAutoGainEnabled;}

    /**
     * @brief isAutoExposureEnabled: Check if the camera auto exposure feature is enabled.
     *
     * @return Boolean. It returns true if the auto exposure feature is enabled.
    */
    bool isAutoExposureEnabled() const {return _isAutoExposureEnabled;}

    /**
     * @brief isAutoWhiteBalanceEnabled: Check if the camera auto white balance
     *  feature is enabled.
     *
     * @return Boolean. It returns true if the auto white balance feature is enabled.
    */
    bool isAutoWhiteBalanceEnabled() const {return _isAutoWhiteBalanceEnabled;}

private:
    /**
     * @brief openCamera: It initializes the Pylon module, and it tries to attach
     *  to the device that has a user-defined name that matches with the given value.
     *  If the given name is an empty string, the system tries to attach to the first
     *  device found.
     *
     * @arg camToConnect: String with the camera user-defined name we want to connect to.
     *  If empty, the first found camera is used.
     *
     * @returns true if no problem is encountered.
    */
    bool openCamera(std::string camToConnect);

    /**
     * @brief initializeCamera: It first checks if it can request some parameters
     *  from the camera. If that's possible, then all the camera parameters
     *  are set to default values.
     *
     * @return Boolean. True if the camera initialization has been done correctly.
    */
    bool initializeCamera();

    /**
     * @brief changeBitDepth: Change the sensor pixel's bit-depth.
     *
     * @arg newBitDepth: Integer value with the new resolution to set (8, 10,
     *  12, etc). The Basler camera only allows 8 and 12  bits.
    */
    void changeBitDepth(int newBitDepth);

    /**
     * @brief initExposureTimeLimits: Function used during initialization only.
     *  It initializes the internal vector with the exposure time limits
     *  from the camera Basler.
    */
    void initExposureTimeLimits();

    /**
     * @brief initGainLimits: Function used during initialization only.
     *  It initializes the internal vector with the gain limits of the Basler camera.
    */
    void initGainLimits();

    /**
     * @brief getSingleImageBytes: Retrieve an image as a raw uint8_t pointer.
     *  This way, we can reduce the time in doing copies of this pointer
     *  when converting it into cv::Mat, for instance. If an OpenCV
     *  image is desired, the the function getSingleImage must be called.
     *
     * @arg imageWidget [output]: Retrieved image width
     * @arg imageWidget [output]: Retrieved image height
     * @arg imageWidget [output]: Retrieved image data pointer.
    */
    void getSingleImageBytes(int &imageWidth, int &imageHeight, uint8_t* &image);

    /// Camera pointer
    std::unique_ptr<Pylon::CInstantCamera> _cam_ptr;

    // Camera gain and exposure time limits vectors
    std::vector<int> _gainLimits;
    std::vector<double> _exposureLimits;

    // Last set values for the camera features.
    bool _isAutoGainEnabled;
    bool _isAutoExposureEnabled;
    bool _isAutoWhiteBalanceEnabled;
    int _lastFrameRate;

    /**
     * NOTE: Available formats for this camera
     *  -) Mono8
     *  -) BayerRG8
     *  -) BayerRG12
     *  -) BayerRG12Packed
     *  -) YUV422_YUYV_Packed
     *  -) RGB8packed
     *  -) BGR8packed
     *
     * Even though it is possible to chose these formats, only BayerRG are
     * valid formats for the polarimetric cameras. The other formats are
     * available since the firmware of the FPGA inside the camera is
     * the same as for the RGB non-polarimetric camera. Selecting BayerRG
     * format, we have all the pixels in the most raw version.
    */
    std::string _chosenFormat;
    int _openCVDatatype;
    bool _isCameraStreaming;
    int _grabbingTimeout_ms;

    const std::string _monoPolaCamModel;
    const std::string _colorPolaCamModel;

    float _targetBrightness;
    int _maximumExposure;
    bool _isFreeRunMode;

    std::string _camToConnect;
};

#endif //__CAMERA_DRIVER_HPP__
