// OpenCV includes

// STD includes
#include <iostream>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/PixelGainCalibration/SourceOrientationEstimator.hpp"

// ROS includes

/**
 * @brief templateAngleEstimation: This function serves to compute the average
 *  AoP from an image, independently of the data type. This avoid future
 *  problems if we decide to change the raw image data type.
 *
 * @arg input: Image from which we want to compute the angle of polarization
 * @arg ROISize: (Height, Width) of the ROI placed around the center of the image.
 *  If Height or Width is negative, then the entire dimension is taken.
 * @arg I135Offset: (Row, Column) offset of the pixel that measures the orientation at 135 degrees
 * @arg I0Offset: (Row, Column) offset of the pixel that measures the orientation at 0 degrees
 * @arg I90Offset: (Row, Column) offset of the pixel that measures the orientation at 90 degrees
 * @arg I45Offset: (Row, Column) offset of the pixel that measures the orientation at 45 degrees
 * @arg Template arg: The datatype of the input matrix
 *
 * @returns: Average angle of polarization of the measured light.
*/
template<typename T>
double templateAngleEstimation(const cv::Mat &input,
    std::vector<int> ROISize,
    std::vector<int> I135Offset,
    std::vector<int> I0Offset,
    std::vector<int> I90Offset,
    std::vector<int> I45Offset)
{
    assert(!(input.empty()));

    int width = static_cast<int>(input.cols);
    int height = static_cast<int>(input.rows);

    int startingRow = 0;
    int endingRow   = height;

    if (ROISize[0] > 0)
    {
        startingRow = (height / 2) - ROISize[0];
        endingRow   = (height / 2) + ROISize[0];
    }

    int startingCol = 0;
    int endingCol   = width;
    if (ROISize[1] > 0)
    {
        startingCol = (width / 2) - ROISize[1];
        endingCol   = (width / 2) + ROISize[1];
    }

    double sinesAvg = 0;
    double cosinesAvg = 0;
    int N = 0;
    for (int i = startingRow; i < endingRow; i += 2)
    {
        for (int j = startingCol; j < endingCol; j += 2)
        {
            /// We retrieve the four pixels of interest
            double I135 = input.at<T>(i + I135Offset[0], j + I135Offset[1]);
            double I0 = input.at<T>(i +   I0Offset[0], j + I0Offset[1]);
            double I90 = input.at<T>(i +  I90Offset[0], j + I90Offset[1]);
            double I45 = input.at<T>(i +  I45Offset[0], j + I45Offset[1]);

            // 2 * Angle = atan2(S2, S1), where S2 = I45 - I135 and S1 = I0 - I90
            float angle = std::atan2(I45 - I135, I0 - I90);
            // This equation is as the running average, but the factors are of the
            // same order. Thus, the error of adding a huge number and a small number
            // are reduced.
            float sine_angle = std::sin(angle);
            float cosine_angle = std::cos(angle);
            sinesAvg =   ((N * (sinesAvg - sine_angle)) + ((N + 1) * sine_angle)) / (N + 1);
            cosinesAvg = ((N * (cosinesAvg - cosine_angle)) + ((N + 1) * cosine_angle)) / (N + 1);
            N++;
        }
    }

    // We divide by 2 and we obtain the average of alpha
    double avgAngle = 0.5 * std::atan2(sinesAvg, cosinesAvg) * 180.0 / M_PI;
    if (avgAngle < 0)
    {
        avgAngle += 180.0;
    }
    return avgAngle;
}

SourceOrientationEstimator::SourceOrientationEstimator(std::vector<int> camFilterOrient, int roiSize) :
    _centerHeight(roiSize),
    _centerWidth(roiSize)
{
    assert(roiSize >= 0 && (!bool(roiSize % 2)));
    updateSPOrientations(camFilterOrient);
}

void SourceOrientationEstimator::updateSPOrientations(std::vector<int> newOrientations)
{
    assert(newOrientations.size() == 4);
    _I135Offset = {static_cast<int>(newOrientations[0] / 2), static_cast<int>(newOrientations[0] % 2)};
    _I0Offset = {static_cast<int>(newOrientations[1] / 2), static_cast<int>(newOrientations[1] % 2)};
    _I90Offset = {static_cast<int>(newOrientations[2] / 2), static_cast<int>(newOrientations[2] % 2)};
    _I45Offset = {static_cast<int>(newOrientations[3] / 2), static_cast<int>(newOrientations[3] % 2)};
}

double SourceOrientationEstimator::getLightSourceOrientation(const cv::Mat &input) const
{
    uchar depth = input.type() & CV_MAT_DEPTH_MASK;
    assert(input.channels() == 1);
    double ret_val = -1;
    switch(depth)
    {
        case CV_8U:
        {
            ret_val = templateAngleEstimation<uchar>(input, {_centerHeight, _centerWidth}, _I135Offset, _I0Offset, _I90Offset, _I45Offset);
            break;
        }
        case CV_16U:
        {
            ret_val = templateAngleEstimation<ushort>(input, {_centerHeight, _centerWidth}, _I135Offset, _I0Offset, _I90Offset, _I45Offset);
            break;
        }
        case CV_32F:
        {
            ret_val = templateAngleEstimation<float>(input, {_centerHeight, _centerWidth}, _I135Offset, _I0Offset, _I90Offset, _I45Offset);
            break;
        }
        case CV_64F:
        {
            ret_val = templateAngleEstimation<double>(input, {_centerHeight, _centerWidth}, _I135Offset, _I0Offset, _I90Offset, _I45Offset);
            break;
        }
        default:
        {
            std::cout << "ERROR: Data type not handled when splitting images" << std::endl;
            assert(0);
            break;
        }
    }
    return ret_val;
}