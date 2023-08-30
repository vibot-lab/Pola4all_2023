// OpenCV includes

// STD includes
#include <iostream>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/PixelGainCalibration/BlackCurrentCorrection.hpp"

// ROS includes

BlackCurrentCorrection::BlackCurrentCorrection() {}

void BlackCurrentCorrection::reset()
{
    // We reinitialize the internal variables
    _calibrationMatrix = cv::Mat();
    _sampleImgs = cv::Mat();
}

void BlackCurrentCorrection::loadData(const std::vector<double>& angles, const cv::Mat& avgImgs)
{
    if (!avgImgs.empty())
    {
        reset();
        _exposuresVector = angles;
        avgImgs.copyTo(_sampleImgs);
    }
}

void BlackCurrentCorrection::correctImage(cv::Mat& rawImg, cv::Mat &output)
{
    if (!rawImg.empty() && !_calibrationMatrix.empty())
    {
        cv::Mat floatImg;
        rawImg.convertTo(floatImg, _calibrationMatrix.type());
        output = floatImg - _calibrationMatrix;
    }
    else
    {
        output = rawImg;
    }
}

void BlackCurrentCorrection::calibrate(std::string outputFilename)
{
    if (!_sampleImgs.empty())
    {
        // For the moment, we use a fixed black current measurements as calibration.
        cv::extractChannel(_sampleImgs, _calibrationMatrix, 0);

        if (!outputFilename.empty())
        {
            // We store the result in a matrix
            // We cannot store images of doubles, so we use FileStorage from OpenCV
            cv::FileStorage calibFS(outputFilename + "/" + "offset_calib_matrix.yaml", cv::FileStorage::WRITE);
            calibFS << "mat1" << _calibrationMatrix;
            calibFS.release();
        }
    }
}
