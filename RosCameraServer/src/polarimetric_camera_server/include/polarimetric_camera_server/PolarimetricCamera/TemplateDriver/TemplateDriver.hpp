#ifndef __TEMPLATE_CLIENT_HPP__
#define __TEMPLATE_CLIENT_HPP__

// OpenCV includes
#include <opencv2/highgui.hpp>

// STD includes
#include <cmath>
#include <iostream>
#include <string>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/IPolarimetricCamera.hpp"

/**
 * @brief TemplateClient class
 *  This class can be used as a model to follow if in the future we want
 * implement a new connection method. This are the minimum functions we need to
 * define. It must implement the pure-virtual methods from IPolarimetricCamera.
*/
class TemplateClient : public IPolarimetricCamera
{
public:
    TemplateClient() :
        IPolarimetricCamera()
    {
        _internalState = {
            .gain = 1,
            .exposureUs = 22,
            .frameRate = 30,
            .redGain = 1,
            .greenGain = 1,
            .blueGain = 1,
            .autoGainEnabled = false,
            .autoExposureEnabled = false,
            .autoWhiteBalanceEnabled = false
        };

        std::string imagePath = std::string(PROJECT_PATH) + "include/polarimetric_camera_server/PolarimetricCamera/TemplateDriver/cars_1.png";
        cv::Mat img = cv::imread(imagePath.c_str(), cv::IMREAD_UNCHANGED | cv::IMREAD_ANYDEPTH);

        if(img.empty())
        {
            std::cout << "Could not read the image: " << imagePath << std::endl;
            return;
        }

        /// We did some mistakes when we stored the images, so some of them are
        // graylevel, but stored as color. Therefore, we check the amount of channels.
        // If the amount of channels is not 1, then we save only the first
        // channel
        uchar chans = 1 + (img.type() >> CV_CN_SHIFT);
        uchar depth = img.type() & CV_MAT_DEPTH_MASK;
        switch(depth)
        {
            case CV_8U:
            {
                _origMaxAllowedVal = 255.0;
                break;
            }
            case CV_16U:
            {
                _origMaxAllowedVal = 4095.0;
                break;
            }
            default:
            {
                std::cerr << "OpenCV datatype not handled" << std::endl;
                assert(0);
                break;
            }
        }

        if (chans == 1)
        {
            _originalInternalImg = img.clone();
        }
        else
        {
            extractChannel(img, _originalInternalImg, 0);
        }
        _originalInternalImg.copyTo(_internalImage);
        _maxAllowedVal = _origMaxAllowedVal;
    }

    virtual ~TemplateClient() {}

    bool startGrabbing(int bitDepth, std::string camToConnect = "") override {(void)camToConnect; changeBitDepth(bitDepth); return true;}
    void stopGrabbing() override {return;}

    bool initializeTriggeringMode(int bitDepth, std::string camToConnect = "") override {(void)camToConnect; changeBitDepth(bitDepth); return true;}

    // Public API: Parameters that can be changed
    // Accessors
    cv::Mat getSingleImage() override
    {
        cv::Mat img = _internalImage * _internalState.gain;
        cv::threshold(img, img, _maxAllowedVal, _maxAllowedVal, cv::THRESH_TRUNC);
        return img;
    }

    std::vector<int> getGainLimits() const override {return std::vector<int>({0, 24});}
    std::vector<double> getExposureLimits() const override {return std::vector<double>({0, 1e+7});}
    float getTemperature() const override {return 10.0f;}
    bool isAlive() const override {return true;}
    int getFrameRate() const override {return _internalState.frameRate;}
    double getExposureTime() const override {return _internalState.exposureUs;}
    double getGain() const override {return 10.0 * std::log(_internalState.gain) / std::log(10.0);}
    std::vector<double> getWhiteBalanceGains() const override {return std::vector<double>({_internalState.redGain, _internalState.greenGain, _internalState.blueGain});}

    bool isAutoGainEnabled() const override {return _internalState.autoGainEnabled;}
    bool isAutoExposureEnabled() const override {return _internalState.autoExposureEnabled;}
    bool isAutoWhiteBalanceEnabled() const override {return _internalState.autoWhiteBalanceEnabled;}

    // Setters
    void changeExposureTime(const CameraState& state) override {_internalState.exposureUs = state.exposureUs;}
    void changeFrameRate(const CameraState& state) override {_internalState.frameRate = state.frameRate;}
    void changeGain(const CameraState& state) override {_internalState.gain = std::exp(std::log(10.0) * state.gain / 10.0);}
    void adjustWhiteBalance(const CameraState& state) override {_internalState.redGain = state.redGain; _internalState.greenGain = state.greenGain; _internalState.blueGain = state.blueGain;}

    void enableAutoExposure(const CameraState& state) override {_internalState.autoExposureEnabled = state.autoExposureEnabled;}
    void enableAutoGain(const CameraState& state) override {_internalState.autoGainEnabled = state.autoGainEnabled;}
    void enableAutoWhiteBalance(const CameraState& state) override {_internalState.autoWhiteBalanceEnabled = state.autoWhiteBalanceEnabled;}

    // Checking camera state

private:
    void changeBitDepth(int newBitDepth)
    {
        if (newBitDepth == 8)
        {
            _originalInternalImg.convertTo(_internalImage, CV_8U, 255.0 / _origMaxAllowedVal);
            _maxAllowedVal = 255.0;
        }
        else if (newBitDepth == 12)
        {
            _originalInternalImg.convertTo(_internalImage, CV_16U, 4095.0 / _origMaxAllowedVal);
            _maxAllowedVal = 4095.0;
        }
        else
        {
            std::cerr << "No valid bit-depth: " << newBitDepth << std::endl;
            assert(0);
        }
    }

    CameraState _internalState;
    cv::Mat _originalInternalImg;
    cv::Mat _internalImage;
    float _maxAllowedVal;
    float _origMaxAllowedVal;
};

#endif // __USB_CAM_CLIENT_HPP__