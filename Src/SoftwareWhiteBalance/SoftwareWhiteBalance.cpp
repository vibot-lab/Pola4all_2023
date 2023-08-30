// OpenCV includes
#include <opencv2/imgproc.hpp>

// STD includes
#include <algorithm>
#include <numeric>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes
#include "SoftwareWhiteBalance/SoftwareWhiteBalance.hpp"

SoftwareWhiteBalance::SoftwareWhiteBalance() :
    _isAutoWhiteBalanceEnabled(false),
    _gains({1,1,1}),
    _rectOfInterest({0,0,10,10}),
    _valueThreshold(220)
{}

SoftwareWhiteBalance::~SoftwareWhiteBalance() {}

void SoftwareWhiteBalance::setRectOfInterest(std::vector<int> topLeftCorner, std::vector<int> bottomRightCorner)
{
    // topLeftCorner and bottomRightCorner must contain 2 elements each, and
    // bottomRightCorner must be at the right-down size of the topLeftCorner
    assert(topLeftCorner.size() == 2);
    assert(bottomRightCorner.size() == 2);
    assert(topLeftCorner[0] < bottomRightCorner[0]);
    assert(topLeftCorner[1] > bottomRightCorner[1]);
    _rectOfInterest = std::vector<int>({topLeftCorner[0], topLeftCorner[1], bottomRightCorner[0], bottomRightCorner[1]});
}

cv::Mat SoftwareWhiteBalance::applyBalance(const cv::Mat &input)
{
    cv::Mat output;
    if (input.channels() == 3)
    {
        cv::multiply(input, cv::Scalar(_gains[0], _gains[1], _gains[2]), output);
    }
    else
    {
        // For the moment, do nothing
        output = input;
    }
    return output.clone();
}

std::vector<cv::Mat> SoftwareWhiteBalance::applyBalance(const std::vector<cv::Mat> &input)
{
    std::vector<cv::Mat> output;
    if (_isAutoWhiteBalanceEnabled && input.size())
    {
        improvedAutoUpdateGains(input[0]);
    }

    for (auto &img : input)
    {
        if (img.channels() == 3)
        {
            output.push_back(img.mul( cv::Scalar(_gains[0], _gains[1], _gains[2]) ));
        }
        else
        {
            // For the moment, do nothing
            output.push_back(img.clone());
        }
    }
    return output;
}

void SoftwareWhiteBalance::enableAutoWhiteBalance(bool enabled)
{
    _isAutoWhiteBalanceEnabled = enabled;
}

void SoftwareWhiteBalance::improvedAutoUpdateGains(const cv::Mat &input)
{
    if (input.channels() != 3)
    {
        return;
    }

    cv::Mat thresholdedImg;
    cv::threshold(input, thresholdedImg, 4000, 0, cv::THRESH_TOZERO_INV);

    std::vector<cv::Mat> channels;
    cv::split(thresholdedImg, channels);

    if (channels.size())
    {
        cv::Mat average = (channels[0] + channels[1] + channels[2]) / 3;

        cv::Point maxLoc;
        double maxVal;
        cv::minMaxLoc(average, NULL, &maxVal, NULL, &maxLoc);

        cv::Vec3s pixelVal = input.at<cv::Vec3s>(maxLoc);

        double maxColor = std::max(std::max(pixelVal[0], pixelVal[1]), pixelVal[2]);
        for (unsigned int i = 0; i < _gains.size(); i++)
        {
            if (pixelVal[i])
            {
                _gains[i] = maxColor / pixelVal[i];
            }
            else
            {
                _gains[i] = 1.0;
            }
        }
    }
}

void SoftwareWhiteBalance::setGains(std::vector<double> gains)
{
    if (!_isAutoWhiteBalanceEnabled)
    {
        _gains = gains;
    }
}
