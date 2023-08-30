// OpenCV includes

// STD includes
#include <iostream>

// Qt Includes

// Custom includes
#include "PolarimetricCamera/BaslerCamDriver/BaslerCamDriver.hpp"

BaslerCamDriver::BaslerCamDriver() :
    IPolarimetricCamera(),
    _isAutoGainEnabled(false),
    _isAutoExposureEnabled(false),
    _isAutoWhiteBalanceEnabled(false),
    _chosenFormat(""),
    _openCVDatatype(-1),
    _lastFrameRate(0),
    _isCameraStreaming(false),
    _grabbingTimeout_ms(2000),
    _monoPolaCamModel("75umPOL"),
    _colorPolaCamModel("75ucPOL"),
    _targetBrightness(0.4),
    _maximumExposure(300000),
    _isFreeRunMode(true)
{
    _cam_ptr.reset(new Pylon::CInstantCamera());
}

BaslerCamDriver::~BaslerCamDriver()
{
    std::cout << "Destructor executed" << std::endl;
}

bool BaslerCamDriver::startGrabbing(int bitDepth, std::string camToConnect)
{
    bool ret_val = false;
    if (!isAlive())
    {
        stopGrabbing();
    }

    ret_val = openCamera(camToConnect) && initializeCamera();
    changeBitDepth(bitDepth);

    if (ret_val && !_cam_ptr->IsGrabbing())
    {
        try
        {
            _cam_ptr->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
            ret_val = true;
        }
        catch(const std::exception& e)
        {
            std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
        }
        catch (const GenICam::RuntimeException &e)
        {
            std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
        }
    }
    _isCameraStreaming = ret_val;
    _isFreeRunMode = true;
    return ret_val;
}

bool BaslerCamDriver::initializeTriggeringMode(int bitDepth, std::string camToConnect)
{
    bool retVal = false;
    if (startGrabbing(bitDepth, camToConnect))
    {
        _cam_ptr->StopGrabbing();
        GenApi::INodeMap& nodemap = _cam_ptr->GetNodeMap();

        /// Change the valid trigger condition (RisingEdge, FallingEdge)
        // Select the Frame Start trigger
        Pylon::CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");

        // Set the trigger activation mode to rising edge
        Pylon::CEnumParameter(nodemap, "TriggerActivation").SetValue("RisingEdge");

        /// If the trigger mode is ON, only triggerred images are taken. If Off, the camera is in free run mode.
        /// FrameStart is the only available mode.
        Pylon::CEnumParameter(nodemap, "TriggerMode").SetValue("On");

        /// Select which trigger line will be used. Our camera has only Line1, Line3 and Line4 as hardware triggers.
        // Set the trigger source to Line 1
        Pylon::CEnumParameter(nodemap, "TriggerSource").SetValue("Line1");

        /// The trigered mode is selected by Trigger mode. The source is Line 1.
        /// Therefore, we place the camera in continuous mode, and it will take
        /// pictures each time we put a trigger signal to the camera.
        /// Additionally, we have to execute the acquision start each time we want
        /// to take a picture, and the camera will wait the trigger signal.
        try
        {
            Pylon::CEnumParameter(nodemap, "AcquisitionMode").SetValue("Continuous");
        }
        catch (const GenICam_3_1_Basler_pylon::AccessException& e)
        {
            std::cout << e.what() << std::endl;
        }

        // Switch on image acquisition
        Pylon::CCommandParameter(nodemap, "AcquisitionStart").Execute();

        try
        {
            _cam_ptr->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        }
        catch (const GenICam_3_1_Basler_pylon::RuntimeException& e)
        {
            std::cout << e.what() << std::endl;
        }
        std::cout << "BaslerCamDriver::initializeTriggeringMode: Trigger intialized" << std::endl;

        _isFreeRunMode = false;
        retVal = true;
    }
    return retVal;
}

void BaslerCamDriver::getSingleImageBytes(int &imageWidth,
    int &imageHeight,
    uint8_t* &image)
{
    imageWidth = 0;
    imageHeight = 0;
    image = nullptr;

    if(!_isCameraStreaming)
    {
        std::cout << "WARNING: Trying to get an image when the camera is not initialized" << std::endl;
        assert(0);
    }

    try
    {
        GenApi::INodeMap& nodemap = _cam_ptr->GetNodeMap();
        // Get the resulting frame rate
        _lastFrameRate = static_cast<int>(0.5 + Pylon::CFloatParameter(nodemap.GetNode("ResultingFrameRate")).GetValue());
        if (_cam_ptr->IsGrabbing() || !_isFreeRunMode)
        {
            Pylon::CGrabResultPtr ptrGrabResult;
            bool isImageGrabbed = false;
            try
            {
                _cam_ptr->RetrieveResult(_grabbingTimeout_ms, ptrGrabResult, Pylon::TimeoutHandling_Return);
                if (ptrGrabResult)
                {
                    isImageGrabbed = true;
                }
            }
            catch(const GenICam::TimeoutException &e)
            {
                std::cerr << __func__ << ":" << __LINE__ << " - "
                          << "Timeout exception happened when trying to grab an image. "
                          << _grabbingTimeout_ms << " mS are not enough to get the image. "
                          << "Returning empty vector" << std::endl;
            }

            if (isImageGrabbed)
            {
                if (ptrGrabResult->GrabSucceeded())
                {
                    imageWidth = static_cast<int>(ptrGrabResult->GetWidth());
                    imageHeight = static_cast<int>(ptrGrabResult->GetHeight());
                    image = static_cast<uint8_t*>(ptrGrabResult->GetBuffer());
                }
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
    }
    catch (const GenICam::RuntimeException &e)
    {
        std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
    }
}

cv::Mat BaslerCamDriver::getSingleImage()
{
    uint8_t* imagePtr;
    int width;
    int height;
    getSingleImageBytes(width, height, imagePtr);
    cv::Mat img;
    if (imagePtr)
    {
        img = cv::Mat(height, width, _openCVDatatype, imagePtr);
    }
    return img;
}

void BaslerCamDriver::stopGrabbing()
{
    _cam_ptr->Close();
    _cam_ptr->DetachDevice();
    Pylon::PylonTerminate();
    _isCameraStreaming = false;
    _lastFrameRate = 0;
}

bool BaslerCamDriver::openCamera(std::string camToConnect)
{
    bool ret_val = false;
    _camToConnect = camToConnect;
    Pylon::PylonInitialize();

    try
    {
        // I know it seems that I am dupplicating the PylonInitialize (look a little above).
        // but without this second call, we cannot retrieve images in a multi-camera mode.
        Pylon::PylonInitialize();

        // Get the transport layer factory.
        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t devices;
        if (tlFactory.EnumerateDevices(devices) == 0)
        {
            std::cout << "ERROR: No Basler camera detected" << std::endl;
        }
        else
        {
            int camIndex = 0;
            std::cout << "Searching for camera with name: " << camToConnect << std::endl;
            while ((!camToConnect.empty()) && (camIndex < devices.size()))
            {
                std::string camName = std::string(devices[camIndex].GetUserDefinedName());
                std::cout << "Camera " << camIndex << ": " << camName << std::endl;
                if (camName == camToConnect)
                {
                    break;
                }
                camIndex++;
            }

            if (camIndex < devices.size())
            {
                std::cout << "Camera found!" << std::endl;
                _cam_ptr->Attach(tlFactory.CreateDevice(devices[camIndex]));
                _cam_ptr->Open();
                ret_val = true;
            }
            else
            {
                std::cout << "ERROR: No Basler camera detected with the name " << camToConnect << std::endl;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
    }
    catch (const GenICam::RuntimeException &e)
    {
        std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
    }
    return ret_val;
}

bool BaslerCamDriver::initializeCamera()
{
    bool ret_val = false;
    if (isAlive())
    {
        GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();

        try
        {
            std::string camName = _cam_ptr->GetDeviceInfo().GetModelName().c_str();
            std::cout << "Using device " << camName << std::endl;

            if (camName.find(_monoPolaCamModel) == std::string::npos)
            {
                /// We disable the light source preset, since they do not work properly in this camera.
                GenApi::CEnumerationPtr(nodemap.GetNode("LightSourcePreset"))->FromString("Off");
            }

            // With these two lines, we make the camera to create triggering signals each time the
            // exposure is active.
            Pylon::CEnumParameter(nodemap, "LineSelector").SetValue("Line2");
            Pylon::CEnumParameter(nodemap, "LineSource").SetValue("ExposureActive");

            initGainLimits();
            initExposureTimeLimits();

            // Set initial gain
            CameraState initState = {
                .gain = 0.0,
                .exposureUs = 15000.0,
                .frameRate = 30,
                .redGain = 1.0,
                .greenGain = 1.0,
                .blueGain = 1.0,
                .autoGainEnabled = false,
                .autoExposureEnabled = false,
                .autoWhiteBalanceEnabled = false
            };
            changeGain(initState);

            // Init gain
            enableAutoGain(initState);

            // Init exposure
            enableAutoExposure(initState);
            changeExposureTime(initState);

            // Set initial frame rate
            changeFrameRate(initState);

            _lastFrameRate = static_cast<int>(0.5 + Pylon::CFloatParameter(nodemap.GetNode("ResultingFrameRate")).GetValue());

            ret_val = true;
        }
        catch(const std::exception& e)
        {
            std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
        }
        catch (const GenICam::RuntimeException &e)
        {
            std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
        }
        catch (const GenICam::InvalidArgumentException &e)
        {
            std::cerr << __func__ << ":" << __LINE__ << " - " << e.what() << std::endl;
        }
    }

    return ret_val;
}

void BaslerCamDriver::changeBitDepth(int newBitDepth)
{
    if (isAlive())
    {
        GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
        std::string camName = _cam_ptr->GetDeviceInfo().GetModelName().c_str();

        if (camName.find(_monoPolaCamModel) != std::string::npos)
        {
            _chosenFormat = "Mono";
        }
        else
        {
            _chosenFormat = "BayerRG";
        }

        switch(newBitDepth)
        {
            case 8:
            {
                _chosenFormat += std::to_string(newBitDepth);
                _openCVDatatype = CV_8UC1;
                break;
            }
            case 12:
            {
                _chosenFormat += std::to_string(newBitDepth);
                _openCVDatatype = CV_16UC1;
                break;
            }
            default:
            {
                std::cout << "No valid bit-depth: " << newBitDepth << std::endl;
                assert(0);
                break;
            }
        }
        Pylon::CEnumParameter(nodemap, "PixelFormat").SetValue(_chosenFormat.c_str());
    }
}

bool BaslerCamDriver::isAlive() const
{
    return _cam_ptr->IsOpen() && !_cam_ptr->IsCameraDeviceRemoved();
}

float BaslerCamDriver::getTemperature() const
{
    float ret_val = 0.0f;
    if (isAlive())
    {
        // Get the current device temperature
        ret_val = Pylon::CFloatParameter(_cam_ptr->GetNodeMap(), "DeviceTemperature").GetValue();
    }
    return ret_val;
}

int BaslerCamDriver::getFrameRate() const
{
    return _lastFrameRate;
}

double BaslerCamDriver::getExposureTime() const
{
    double ret_val = 0.0;
    if (isAlive())
    {
        // Determine the current exposure time
        ret_val = Pylon::CFloatParameter(_cam_ptr->GetNodeMap(), "ExposureTime").GetValue();
    }
    return ret_val;
}

double BaslerCamDriver::getGain() const
{
    double ret_val = 0.0;
    if (isAlive())
    {
        // Determine the current gain
        ret_val = Pylon::CFloatParameter(_cam_ptr->GetNodeMap(), "Gain").GetValue();
    }
    return ret_val;
}

std::vector<double> BaslerCamDriver::getWhiteBalanceGains() const
{
    std::vector<double> myGains(3, 0);
    std::string camName = _cam_ptr->GetDeviceInfo().GetModelName().c_str();
    if (camName.find(_monoPolaCamModel) == std::string::npos)
    {
        if (isAlive())
        {
            GenApi::INodeMap& nodemap = _cam_ptr->GetNodeMap();
            Pylon::CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Red");
            myGains[0] = Pylon::CFloatParameter(nodemap, "BalanceRatio").GetValue();

            Pylon::CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Green");
            myGains[1] = Pylon::CFloatParameter(nodemap, "BalanceRatio").GetValue();

            Pylon::CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Blue");
            myGains[2] = Pylon::CFloatParameter(nodemap, "BalanceRatio").GetValue();
        }
    }

    return myGains;
}

void BaslerCamDriver::changeExposureTime(const CameraState& state)
{
    double exposure_us = state.exposureUs;

    if (isAlive())
    {
        GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
        if (exposure_us >= _exposureLimits[0] && exposure_us <= _exposureLimits[1])
        {
            if (_isAutoExposureEnabled)
            {
                CameraState dummy;
                dummy.autoExposureEnabled = false;
                enableAutoExposure(dummy);
            }

            // Set the exposure time
            GenApi::CFloatPtr(nodemap.GetNode("ExposureTime"))->SetValue(exposure_us);
        }
        else
        {
            std::cerr << "Desired exposure time out of limits. Min value: " << _exposureLimits[0] << ". Max value: " << _exposureLimits[1] << std::endl;
        }
    }
    else
    {
        throw std::runtime_error("In order to change the exposure, the camera should be connected");
    }
}

void BaslerCamDriver::changeFrameRate(const CameraState& state)
{
    int new_frame_rate = state.frameRate;
    assert(new_frame_rate > 0);
    if (isAlive())
    {
        GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
        GenApi::CBooleanPtr(nodemap.GetNode("AcquisitionFrameRateEnable"))->SetValue(true);
        GenApi::CFloatPtr(nodemap.GetNode("AcquisitionFrameRate"))->SetValue(new_frame_rate);
        _lastFrameRate = new_frame_rate;
    }
}

void BaslerCamDriver::changeGain(const CameraState& state)
{
    double new_gain = state.gain;
    assert(new_gain >= _gainLimits[0] && new_gain <= _gainLimits[1]);
    if (isAlive())
    {
        if (_isAutoGainEnabled)
        {
            CameraState dummy;
            dummy.autoGainEnabled = false;
            enableAutoGain(dummy);
        }

        GenApi::CFloatPtr(_cam_ptr->GetNodeMap().GetNode("Gain"))->SetValue(new_gain);
    }
    else
    {
        throw std::runtime_error("In order to change the gain, the camera should be connected");
    }
}

void BaslerCamDriver::enableAutoExposure(const CameraState& state)
{
    bool enabled = state.autoExposureEnabled;
    if (isAlive())
    {
        GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
        if (enabled)
        {
            GenApi::CFloatPtr(nodemap.GetNode("AutoExposureTimeLowerLimit"))->SetValue(_exposureLimits[0]);
            GenApi::CFloatPtr(nodemap.GetNode("AutoExposureTimeUpperLimit"))->SetValue(_exposureLimits[1]);

            // Set the target brightness value
            GenApi::CFloatPtr(nodemap.GetNode("AutoTargetBrightness"))->SetValue(_targetBrightness);

            // Select auto function ROI 1
            GenApi::CEnumerationPtr(nodemap.GetNode("AutoFunctionROISelector"))->FromString("ROI1");

            // Enable the 'Brightness' auto function (Gain Auto + Exposure Auto)
            // for the auto function ROI selected
            GenApi::CBooleanPtr(nodemap.GetNode("AutoFunctionROIUseBrightness"))->SetValue(true);

            // Enable Exposure Auto by setting the operating mode to Continuous
            GenApi::CEnumerationPtr(nodemap.GetNode("ExposureAuto"))->FromString("Continuous");
            _isAutoExposureEnabled = true;
        }
        else
        {
            GenApi::CEnumerationPtr(nodemap.GetNode("ExposureAuto"))->FromString("Off");
            _isAutoExposureEnabled = false;
        }
    }
}

void BaslerCamDriver::enableAutoGain(const CameraState& state)
{
    bool enabled = state.autoGainEnabled;
    if (isAlive())
    {
        GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
        if (enabled)
        {
            GenApi::CFloatPtr(nodemap.GetNode("AutoGainLowerLimit"))->SetValue(_gainLimits[0]);
            GenApi::CFloatPtr(nodemap.GetNode("AutoGainUpperLimit"))->SetValue(_gainLimits[1]);

            // Specify the target value
            GenApi::CFloatPtr(nodemap.GetNode("AutoTargetBrightness"))->SetValue(_targetBrightness);

            // Select auto function ROI 1
            GenApi::CEnumerationPtr(nodemap.GetNode("AutoFunctionROISelector"))->FromString("ROI1");

            // Enable the 'Brightness' auto function (Gain Auto + Exposure Auto)
            // for the auto function ROI selected
            GenApi::CBooleanPtr(nodemap.GetNode("AutoFunctionROIUseBrightness"))->SetValue(true);

            // Enable Gain Auto by setting the operating mode to Continuous
            GenApi::CEnumerationPtr(nodemap.GetNode("GainAuto"))->FromString("Continuous");
            _isAutoGainEnabled = true;
        }
        else
        {
            GenApi::CEnumerationPtr(nodemap.GetNode("GainAuto"))->FromString("Off");
            _isAutoGainEnabled = false;
        }
    }
}

void BaslerCamDriver::enableAutoWhiteBalance(const CameraState& state)
{
    bool enabled = state.autoWhiteBalanceEnabled;
    if (isAlive())
    {
        std::string camName = _cam_ptr->GetDeviceInfo().GetModelName().c_str();
        if (camName.find(_monoPolaCamModel) == std::string::npos)
        {
            GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
            if (enabled)
            {
                // Select auto function ROI 2
                Pylon::CEnumParameter(nodemap, "AutoFunctionROISelector").SetValue("ROI2");
                // Enable the Balance White Auto auto function
                // for the auto function ROI selected
                Pylon::CBooleanParameter(nodemap, "AutoFunctionROIUseWhiteBalance").SetValue(true);
                // Enable Balance White Auto by setting the operating mode to Continuous
                Pylon::CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Continuous");

                _isAutoWhiteBalanceEnabled = true;
            }
            else
            {
                Pylon::CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Off");
                _isAutoWhiteBalanceEnabled = false;
            }
        }
    }
}

void BaslerCamDriver::adjustWhiteBalance(const CameraState& state)
{
    double redGain = state.redGain;
    double greenGain = state.greenGain;
    double blueGain = state.blueGain;

    assert(redGain >= 1 && greenGain >= 1 && blueGain >= 1);
    if (isAlive())
    {
        std::string camName = _cam_ptr->GetDeviceInfo().GetModelName().c_str();
        if (camName.find(_monoPolaCamModel) == std::string::npos)
        {
            if (_isAutoWhiteBalanceEnabled)
            {
                CameraState dummy;
                dummy.autoWhiteBalanceEnabled = false;
                enableAutoWhiteBalance(dummy);
            }
            GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();

            Pylon::CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Red");
            Pylon::CFloatParameter(nodemap, "BalanceRatio").SetValue(redGain);

            Pylon::CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Green");
            Pylon::CFloatParameter(nodemap, "BalanceRatio").SetValue(greenGain);

            Pylon::CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Blue");
            Pylon::CFloatParameter(nodemap, "BalanceRatio").SetValue(blueGain);
        }
    }
}

void BaslerCamDriver::initExposureTimeLimits()
{
    GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
    double minLowerLimit = GenApi::CFloatPtr(nodemap.GetNode("AutoExposureTimeLowerLimit"))->GetMin() + 1;
    double maxUpperLimit = GenApi::CFloatPtr(nodemap.GetNode("AutoExposureTimeUpperLimit"))->GetMax();
    if (_maximumExposure > maxUpperLimit)
    {
        _maximumExposure = maxUpperLimit;
    }
    _exposureLimits.clear();
    _exposureLimits.push_back(minLowerLimit);
    _exposureLimits.push_back(_maximumExposure);
}

void BaslerCamDriver::initGainLimits()
{
    GenApi::INodeMap &nodemap = _cam_ptr->GetNodeMap();
    // Set the the Gain Auto auto function to its minimum lower limit
    // and its maximum upper limit
    double minLowerLimit = GenApi::CFloatPtr(nodemap.GetNode("AutoGainLowerLimit"))->GetMin();
    // In order to use analog gains only, we limit the gain to 24, instead of 32
    // double maxUpperLimit = GenApi::CFloatPtr(nodemap.GetNode("AutoGainUpperLimit"))->GetMax();
    double maxUpperLimit = 24;
    _gainLimits.clear();
    _gainLimits.push_back(minLowerLimit);
    _gainLimits.push_back(maxUpperLimit);
}