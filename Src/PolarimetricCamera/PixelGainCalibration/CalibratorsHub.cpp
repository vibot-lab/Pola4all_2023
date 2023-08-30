// OpenCV includes

// STD includes
#include <algorithm>
#include <iostream>

// Qt Includes

// Pylon includes

// Custom includes
#include "CameraTypes.hpp"
#include "PolarimetricCamera/PixelGainCalibration/CalibratorsHub.hpp"
#include "PolarimetricCamera/PixelGainCalibration/SourceOrientationEstimator.hpp"

// ROS includes

CalibratorsHub::CalibratorsHub() :
    _calibratorsNames(NUM_CALIBRATORS)
{
    _calibratorsNames[PIXEL_GAIN] = "Pixel gain";
    _calibratorsNames[BLACK_CURRENT] = "Black current (offset)";
}

std::string CalibratorsHub::getCalibratorName(Calibrators id) const
{
    assert(id < NUM_CALIBRATORS);
    return _calibratorsNames[id];
}

bool CalibratorsHub::isDataLoaded(std::string selectedCalib) const
{
    int idx = getCalibratorIndex(selectedCalib);
    bool ret_val;
    switch (idx)
    {
        case PIXEL_GAIN:
        {
            ret_val = _gainCalibrator.isDataLoaded();
            break;
        }
        case BLACK_CURRENT:
        {
            ret_val = _offsetCalibrator.isDataLoaded();
            break;
        }
        default:
        {
            ret_val = false;
            break;
        }
    }
    return ret_val;
}

bool CalibratorsHub::areCalibrationComputed(std::string selectedCalib) const
{
    int idx = getCalibratorIndex(selectedCalib);
    bool ret_val;
    switch (idx)
    {
        case PIXEL_GAIN:
        {
            ret_val = _gainCalibrator.areCalibrationComputed();
            break;
        }
        case BLACK_CURRENT:
        {
            ret_val = _offsetCalibrator.areCalibrationComputed();
            break;
        }
        default:
        {
            ret_val = false;
            break;
        }
    }
    return ret_val;
}

void CalibratorsHub::loadData(std::string selectedCalib,
    const std::vector<int> camFilterOrient,
    const std::vector<double>& horAxis,
    const cv::Mat& avgImgs)
{
    int idx = getCalibratorIndex(selectedCalib);
    switch (idx)
    {
        case PIXEL_GAIN:
        {
            bool useGTAngles = false;
            std::vector<double> lightAngles;
            cv::Mat samplesImg;
            if (useGTAngles)
            {
                lightAngles = horAxis;
                samplesImg = avgImgs;
            }
            else
            {
                SourceOrientationEstimator anglesEstimator(camFilterOrient, 50);
                std::vector<std::pair<double, int>> mergedInfo(avgImgs.channels());
                std::vector<cv::Mat> chs;
                cv::split(avgImgs, chs);

                // We store the indexes of the channels to save space
                // dynamically used.
                std::cout << "Angles comparison:" << std::endl;
                for (int i = 0; i < avgImgs.channels(); i++)
                {
                    mergedInfo[i] = std::pair<double, int>(anglesEstimator.getLightSourceOrientation(chs[i]), i);
                    std::cout << "GT angle: " << horAxis[i] << " - Estimated angle: " << mergedInfo[i].first << " - Corrected angle: " << mergedInfo[i].first - mergedInfo[0].first << std::endl;
                }

                // We sort the pairs (angles, idxs), based on the angles
                std::sort(mergedInfo.begin(),
                          mergedInfo.end(),
                          [](const std::pair<double, int> &v1, const std::pair<double, int> &v2){
                              return v1.first < v2.first;
                          });

                std::vector<cv::Mat> sortedImgs(chs.size());
                lightAngles.clear();
                for (size_t i = 0; i < mergedInfo.size(); i++)
                {
                    lightAngles.push_back(mergedInfo[i].first);
                    sortedImgs[i] = chs[mergedInfo[i].second];
                }

                cv::merge(sortedImgs, samplesImg);

                // We clear this vector as soon we do not require it anymore.
                chs.clear();
                sortedImgs.clear();
            }
            _gainCalibrator.loadData(lightAngles, samplesImg);
            break;
        }
        case BLACK_CURRENT:
        {
            _offsetCalibrator.loadData(horAxis, avgImgs);
            break;
        }
        default:
        {
            break;
        }
    }
}

std::vector<double> CalibratorsHub::getHorizontalAxis(std::string selectedCalib) const
{
    int idx = getCalibratorIndex(selectedCalib);
    std::vector<double> output;
    switch (idx)
    {
        case PIXEL_GAIN:
        {
            output = _gainCalibrator.getAnglesVector();
            break;
        }
        case BLACK_CURRENT:
        {
            output = _offsetCalibrator.getExposuresVector();
            break;
        }
        default:
        {
            break;
        }
    }
    return output;
}

const cv::Mat& CalibratorsHub::getRawSamples(std::string selectedCalib) const
{
    int idx = getCalibratorIndex(selectedCalib);
    switch (idx)
    {
        case PIXEL_GAIN:
        {
            return _gainCalibrator.getSamples();
            break;
        }
        case BLACK_CURRENT:
        {
            return _offsetCalibrator.getSamples();
            break;
        }
        default:
        {
            assert(0);
            break;
        }
    }
}

void CalibratorsHub::computeCalibrationMatrices(bool useSpCalib)
{
    _gainCalibrator.computeParametersMatrices(useSpCalib);
    // We disable storing the data, since we are not using it yet
    // std::string path = _gainExperimentsSaverLoader.getLastExperimentPath();
    std::string path("");
    _offsetCalibrator.calibrate(path);
}

void CalibratorsHub::correctImage(cv::Mat rawImg, bool correct, cv::Mat &output)
{
    if (correct)
    {
        /// We ignore the offset calibration, since its effect is negligible,
        // and it increases the pipeline processing time.
        // _offsetCalibrator.correctImage(rawImg, rawImg);
        _gainCalibrator.correctImage(rawImg, output);
    }
    else
    {
        rawImg.copyTo(output);
    }
}

int CalibratorsHub::getCalibratorIndex(std::string selectedCalib) const
{
    std::vector<std::string>::const_iterator output = std::find(_calibratorsNames.begin(), _calibratorsNames.end(), selectedCalib);
    int idx = -1;
    if (output != _calibratorsNames.end())
    {
        idx = output - _calibratorsNames.begin();
    }
    else
    {
        assert(0);
    }

    return idx;
}