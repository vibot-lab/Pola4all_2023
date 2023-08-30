// OpenCV includes
#include <opencv2/core/persistence.hpp>

// STD includes
#include <algorithm>
#include <cmath>
#include <iostream>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/PixelGainCalibration/SuperPixelCalibration.hpp"

// ROS includes

constexpr double PiDiv180 = M_PI / 180;

SuperPixelCalibration::SuperPixelCalibration() :
    _lightDoP(std::vector<double>(3,0)),
    _lightS0(std::vector<double>(3,0)),
    _lightAllValues(std::vector<std::vector<std::pair<double, double>>>(3)),
    _N(std::vector<int>(3,0)),
    _roiHW({50,50}),
    _defaultOrientations({135, 0, 90, 45})
{
    //  This camera provides a polarized, colored image in this disposition:
    //                   || R_135 | R_0  || G_135 | G_0  ||
    //                   || R_90  | R_45 || G_90  | G_45 ||
    //                   || G_135 | G_0  || B_135 | B_0  ||
    //                   || G_90  | G_45 || B_90  | B_45 ||
    // Red pixels
    _colorLabels[std::pair<int,int>(0,0)] = 0;
    _colorLabels[std::pair<int,int>(0,1)] = 0;
    _colorLabels[std::pair<int,int>(1,0)] = 0;
    _colorLabels[std::pair<int,int>(1,1)] = 0;

    // Green Pixels
    _colorLabels[std::pair<int,int>(0,2)] = 1;
    _colorLabels[std::pair<int,int>(0,3)] = 1;
    _colorLabels[std::pair<int,int>(1,2)] = 1;
    _colorLabels[std::pair<int,int>(1,3)] = 1;
    _colorLabels[std::pair<int,int>(2,0)] = 1;
    _colorLabels[std::pair<int,int>(3,0)] = 1;
    _colorLabels[std::pair<int,int>(2,1)] = 1;
    _colorLabels[std::pair<int,int>(3,1)] = 1;

    // Blue Pixels
    _colorLabels[std::pair<int,int>(2,2)] = 2;
    _colorLabels[std::pair<int,int>(2,3)] = 2;
    _colorLabels[std::pair<int,int>(3,2)] = 2;
    _colorLabels[std::pair<int,int>(3,3)] = 2;
}

void SuperPixelCalibration::reset()
{
    _allImagesSamples = cv::Mat();
    _anglesVector.clear();

    _phasesImage.release();
    _TiImage.release();
    _PiImage.release();
    _calibrationMatrix.clear();
    _lightDoP.clear();
    _lightS0.clear();
    _N.clear();
    _lightAllValues.clear();
}

bool SuperPixelCalibration::isDataLoaded() const
{
    return !_allImagesSamples.empty();
}

bool SuperPixelCalibration::areCalibrationComputed() const
{
    return !_phasesImage.empty() &&
           !_TiImage.empty() &&
           !_PiImage.empty();
}

void SuperPixelCalibration::loadData(const std::vector<double>& angles, const cv::Mat& avgImgs)
{
    if (!avgImgs.empty())
    {
        reset();
        _anglesVector = angles;
        avgImgs.copyTo(_allImagesSamples);
    }
}

void SuperPixelCalibration::initializeMatricesSizes(int rows, int cols)
{
    _phasesImage = cv::Mat::zeros(cv::Size(cols, rows), CV_64FC1);
    _TiImage = cv::Mat::zeros(cv::Size(cols, rows), CV_64FC1);
    _PiImage = cv::Mat::zeros(cv::Size(cols, rows), CV_64FC1);
    _calibrationMatrix.clear();

    _lightDoP = std::vector<double>(3,0);
    _lightS0 = std::vector<double>(3,0);
    /// While computing the parameters, we compute also the averages of the
    // light intensity and the DoP, considering the pixels as ideals
    _N = std::vector<int>(3, 0);
    _lightAllValues = std::vector<std::vector<std::pair<double, double>>>(3);
}

void SuperPixelCalibration::addLightParamsSample(double TiPiS0, double TiS0DOP, int colorIdx, LightEstimationMethod method)
{
    double idealTi = 0.5;
    double idealPi = 1.0;
    double S0 = TiPiS0 * idealPi / idealTi;
    double dop = TiS0DOP / (idealPi * TiPiS0);

    switch(method)
    {
        case METHOD_AVERAGE:
        {
            /// We do the running average to avoid overflow
            // This equation is as the running average, but the factors are of the
            // same order. Thus, the error of adding a huge number and a small number
            // is reduced.
            _lightS0[colorIdx] = ((_N[colorIdx] * (_lightS0[colorIdx] - S0)) + ((_N[colorIdx] + 1) * S0)) / (_N[colorIdx] + 1);
            _lightDoP[colorIdx] = ((_N[colorIdx] * (_lightDoP[colorIdx] - dop)) + ((_N[colorIdx] + 1) * dop)) / (_N[colorIdx] + 1);
            _N[colorIdx]++;
            break;
        }
        case METHOD_MAXIMUM:
        {
            // We look for the maximum value of the intensity. We store the values
            // of intensity and DoLP of the maximum measurement.
            if (S0 > _lightS0[colorIdx])
            {
                _lightS0[colorIdx] = S0;
                _lightDoP[colorIdx] = dop;
            }
            break;
        }
        case METHOD_MEDIAN:
        {
            // To avoid huge delays, we only store the detected values, and
            // we look the median value at the end only.
            _lightAllValues[colorIdx].push_back(std::pair<double, double>(S0, dop));
            break;
        }
        default:
        {
            std::cerr << "ERROR: Light estimation method not recognized" << std::endl;
            assert(0);
            break;
        }
    }
}

void SuperPixelCalibration::computeLightParamsMedian()
{
    for (unsigned int colorIdx = 0; colorIdx < _lightAllValues.size(); colorIdx++)
    {
        if (_lightAllValues[colorIdx].size())
        {
            double medianS0 = 0;
            double medianDoP = 0;
            int n = _lightAllValues[colorIdx].size() / 2;
            std::nth_element(
                _lightAllValues[colorIdx].begin(),
                _lightAllValues[colorIdx].begin() + n,
                _lightAllValues[colorIdx].end(),
                [](const std::pair<double, double>& v1, const std::pair<double, double>& v2) {return v1.first < v2.first;} );

            medianS0 = _lightAllValues[colorIdx][n].first;
            medianDoP = _lightAllValues[colorIdx][n].second;

            // If we have an even amount of elements, we do the average of the two
            // middle elements
            if (!(_lightAllValues[colorIdx].size() % 2))
            {
                std::nth_element(
                    _lightAllValues[colorIdx].begin(),
                    _lightAllValues[colorIdx].begin() + n - 1,
                    _lightAllValues[colorIdx].end(),
                    [](const std::pair<double, double>& v1, const std::pair<double, double>& v2) {return v1.first < v2.first;} );
                medianS0 =  (medianS0 + _lightAllValues[colorIdx][n + 1].first) / 2.0;
                medianDoP = (medianDoP + _lightAllValues[colorIdx][n + 1].second) / 2.0;
            }
            _lightS0[colorIdx] = medianS0;
            _lightDoP[colorIdx] = medianDoP;
        }
    }
}

void SuperPixelCalibration::computeParametersMatrices(bool useSuperPixel)
{
    if (!_allImagesSamples.empty())
    {
        int rows = _allImagesSamples.rows;
        int cols = _allImagesSamples.cols;
        int channels = _allImagesSamples.channels();
        // This datatype is required to correctly read the original image
        // pointer. Then, we will convert each row to double datatype to be able
        // to do the matrix operations.
        uchar depth = _allImagesSamples.type() & CV_MAT_DEPTH_MASK;
        int pixelDataType = CV_MAKETYPE(depth, 1);

        LightEstimationMethod estMethod = METHOD_MEDIAN;

        initializeMatricesSizes(rows, cols);

        // The Pseudo inverse of the angle samples is fixed, so we compute it once
        cv::Mat samplesPseudo = computeSamplesPseudoInverse(_anglesVector);

        cv::Mat TiPiS0Image(rows, cols, CV_64FC1);
        cv::Mat TiS0DOPImage(rows, cols, CV_64FC1);

        int deltaRows = 1;
        int deltaCols = 1;
        unsigned int amountPixels = 1;
        if (useSuperPixel)
        {
            deltaRows = 2;
            deltaCols = 2;
            amountPixels = 4;
            std::cout << "Using Super pixel calibration" << std::endl;
        }
        else
        {
            std::cout << "Using Single pixel calibration" << std::endl;
        }

        // startingRow, endingRow, startingCol, and endingCol are the limits
        // of a rectangle around the center, used to compute the DoLP and the intensity
        // parameters later used in the calibration.
        int startingRow = 0;
        int endingRow   = rows;
        if (_roiHW[0] > 0)
        {
            startingRow = (rows / 2) - _roiHW[0];
            endingRow   = (rows / 2) + _roiHW[0];
        }

        int startingCol = 0;
        int endingCol   = cols;
        if (_roiHW[1] > 0)
        {
            startingCol = (cols / 2) - _roiHW[1];
            endingCol   = (cols / 2) + _roiHW[1];
        }

        for(int i = 0; i < rows; i += deltaRows)
        {
            for(int j = 0; j < cols; j += deltaCols)
            {
                std::vector<int> rowsIdx;
                std::vector<int> colsIdx;

                if (useSuperPixel)
                {
                    // This array arranges the samples from 0 until 3.
                    // It does not care about the filter orientations.
                    rowsIdx = {   i,     i,   i + 1,  i + 1};
                    colsIdx = {   j,   j + 1,   j,    j + 1};
                }
                else
                {
                    rowsIdx = {i};
                    colsIdx = {j};
                }

                assert(rowsIdx.size() == colsIdx.size());
                /// We fill the samples image with the sample values.
                cv::Mat samples(amountPixels, channels, CV_64FC1);

                for(size_t r = 0; r < rowsIdx.size(); r++)
                {
                    cv::Mat(1, channels, pixelDataType, _allImagesSamples.ptr(rowsIdx[r], colsIdx[r])).convertTo(samples.row(r), CV_64FC1);
                }

                SuperPixelParams optimizedParams;

                optimize(samplesPseudo, samples, optimizedParams);

                // Sanity check
                assert(amountPixels == optimizedParams.TiPiS0.size() &&
                    amountPixels == optimizedParams.TiS0DOP.size() &&
                    amountPixels == optimizedParams.phaseShift.size());

                int colorIdx = getColorIndex(i, j);
                for (unsigned r = 0; r < amountPixels; r++)
                {
                    if ((i >= startingRow && i < endingRow) && (j >= startingCol && j < endingCol))
                    {
                        /// NOTE: you can change the method to change the light estimation value.
                        addLightParamsSample(optimizedParams.TiPiS0[r], optimizedParams.TiS0DOP[r], colorIdx, estMethod);
                    }

                    TiPiS0Image.at<double>(rowsIdx[r], colsIdx[r]) = optimizedParams.TiPiS0[r];
                    TiS0DOPImage.at<double>(rowsIdx[r], colsIdx[r]) = optimizedParams.TiS0DOP[r];
                    _phasesImage.at<double>(rowsIdx[r], colsIdx[r]) = optimizedParams.phaseShift[r];
                }
            }
        }

        if (estMethod == METHOD_MEDIAN)
        {
            computeLightParamsMedian();
        }

        std::cout << "S0 found: [" << _lightS0[0] << ", " << _lightS0[1] << ", " << _lightS0[2] << "]"<< std::endl;
        std::cout << "DoP found: [" << _lightDoP[0] << ", " << _lightDoP[1] << ", " << _lightDoP[2] << "]"<< std::endl;

        for(int i = 0; i < rows; i++)
        {
            for(int j = 0; j < cols; j++)
            {
                int colorIdx = getColorIndex(i, j);
                _TiImage.at<double>(i,j) = TiS0DOPImage.at<double>(i,j) / (_lightS0[colorIdx] * _lightDoP[colorIdx]);
                _PiImage.at<double>(i,j) = TiS0DOPImage.at<double>(i,j) / (TiPiS0Image.at<double>(i,j) * _lightDoP[colorIdx]);
            }
        }

        computeCorrectionMatrix();

        std::cout << "Parameters computation finished" << std::endl;
    }
}

int SuperPixelCalibration::getColorIndex(int row, int col)
{
    /// We know the pixel pattern has a size of 4x4, so we map the given row and
    // columns with the first pattern positions.
    int patternRow = row % 4;
    int patternCol = col % 4;
    return _colorLabels.at(std::pair<int,int>(patternRow, patternCol));
}

cv::Mat SuperPixelCalibration::computeSamplesPseudoInverse(std::vector<double>& angles)
{
    cv::Mat stokesMatrix = cv::Mat::ones(3, angles.size(), CV_64FC1);

    for (size_t i = 0; i < angles.size(); i++)
    {
        double angleInRad = angles[i] * PiDiv180;
        stokesMatrix.at<double>(1, i) = std::cos(2.0 * angleInRad);
        stokesMatrix.at<double>(2, i) = std::sin(2.0 * angleInRad);
    }

    cv::Mat pseudoInverse;
    computePseudoInverse(stokesMatrix, false, pseudoInverse);

    return pseudoInverse;
}

void SuperPixelCalibration::saveCalibrationImage(cv::Mat& img, std::string filepath)
{
    if (!filepath.empty() && !img.empty())
    {
        // We store the result in a matrix
        // We cannot store images of doubles, so we use FileStorage from OpenCV
        cv::FileStorage fileSt(filepath, cv::FileStorage::WRITE);
        fileSt << "mat1" << img;
        fileSt.release();
    }
}

void SuperPixelCalibration::correctImage(cv::Mat& rawImg, cv::Mat &output)
{
    if (!_calibrationMatrix.empty())
    {
        // Sanity check
        assert(rawImg.channels() == 1);

        int rows = rawImg.rows;
        int cols = rawImg.cols;

        // We convert the input image into floating point to increase the computation accuracy.
        cv::Mat floatImg;
        rawImg.convertTo(floatImg, CV_64F);
        // We start with a double floating point accuracy in the output image,
        // and at the end, we will convert it back to the input image type.
        output = cv::Mat::zeros(rows, cols, CV_64FC1);

        assert((rows == _calibrationMatrix[0].rows) && (_calibrationMatrix[0].cols == cols));

        for (int i = 0; i < rows; i+=2)
        {
            for (int j = 0; j < cols; j+=2)
            {
                std::vector<int> rowIdxs({i, i, i + 1, i + 1});
                std::vector<int> colIdxs({j, j + 1, j, j + 1});

                // Raw intensities measurements
                double I0 = floatImg.at<double>(rowIdxs[0], colIdxs[0]);
                double I1 = floatImg.at<double>(rowIdxs[1], colIdxs[1]);
                double I2 = floatImg.at<double>(rowIdxs[2], colIdxs[2]);
                double I3 = floatImg.at<double>(rowIdxs[3], colIdxs[3]);

                for (size_t r = 0; r < rowIdxs.size(); r++)
                {
                    // Each element of the output image is a linear combination of the original
                    // intensities, and the calibration matrix coefficients.
                    output.at<double>(rowIdxs[r], colIdxs[r]) =
                        I0 * _calibrationMatrix[0].at<double>(rowIdxs[r], colIdxs[r]) +
                        I1 * _calibrationMatrix[1].at<double>(rowIdxs[r], colIdxs[r]) +
                        I2 * _calibrationMatrix[2].at<double>(rowIdxs[r], colIdxs[r]) +
                        I3 * _calibrationMatrix[3].at<double>(rowIdxs[r], colIdxs[r]);
                }
            }
        }
        output.convertTo(output, rawImg.type());
    }
    else
    {
        output = rawImg;
    }
}

void SuperPixelCalibration::optimize(const cv::Mat& stokesPseudoInv,
    const cv::Mat& samples,
    SuperPixelParams& cameraParams)
{
    double precision = 1e10;
    cv::Mat res = samples * stokesPseudoInv;
    cameraParams.TiPiS0 = std::vector<double>(res.rows);
    cameraParams.TiS0DOP = std::vector<double>(res.rows);
    cameraParams.phaseShift = std::vector<double>(res.rows);

    for (int i = 0; i < res.rows; i++)
    {
        double theta = std::atan2(res.at<double>(i, 2), res.at<double>(i, 1)) * 0.5 / PiDiv180;
        if (theta < 0)
        {
            theta += 180.0;
        }
        cameraParams.TiPiS0[i] = res.at<double>(i, 0);
        cameraParams.TiS0DOP[i] = std::sqrt((res.at<double>(i, 1) * res.at<double>(i, 1)) + (res.at<double>(i, 2) * res.at<double>(i, 2)));

        // We remove small digits due to quantization problems. For instance,
        // if precision is 1e10, we will leave the first 10 digits of the number,
        // and we clear the rest.
        theta = static_cast<int64_t>(theta * precision) / precision;
        cameraParams.phaseShift[i] = theta;
    }
}

void SuperPixelCalibration::updateFilterOrientations(std::vector<int>& newOrientations)
{
    assert(newOrientations.size() == 4);
    _defaultOrientations = newOrientations;
    _calibrationMatrix.clear();
    // We check if we have to recompute the calibration matrices.
    if (!_phasesImage.empty())
    {
        computeCorrectionMatrix();
    }
}

void SuperPixelCalibration::computeCorrectionMatrix()
{
    if (!_phasesImage.empty())
    {
        /// Sanity check
        assert((_phasesImage.rows == _TiImage.rows) && (_phasesImage.rows == _PiImage.rows) && (_TiImage.rows == _PiImage.rows));
        assert((_phasesImage.cols == _TiImage.cols) && (_phasesImage.cols == _PiImage.cols) && (_TiImage.cols == _PiImage.cols));

        int imgRows = _phasesImage.rows;
        int imgCols = _phasesImage.cols;

        _calibrationMatrix.clear();
        _calibrationMatrix.resize(4);

        for (cv::Mat& ch : _calibrationMatrix)
        {
            ch = cv::Mat(imgRows, imgCols, CV_64FC1);
        }

        for (int i = 0; i < imgRows; i += 2)
        {
            for (int j = 0; j < imgCols; j += 2)
            {
                // Simple assignmed of the result of getIntensityCoeff to
                // stokesMatrix.row(k) does not work. using copyTo is the suggested method
                // by OpenCV
                cv::Mat stokesMatrix(4, 3, CV_64FC1);
                cv::Mat idealMatrix(4, 3, CV_64FC1);

                std::vector<int> rowIdxs({i,   i  , i + 1, i + 1});
                std::vector<int> colIdxs({j, j + 1,   j  , j + 1});
                for (size_t r = 0; r < rowIdxs.size(); r++)
                {
                    getIntensityCoeff(
                        _phasesImage.at<double>(rowIdxs[r], colIdxs[r]),
                        _TiImage.at<double>(rowIdxs[r], colIdxs[r]),
                        _PiImage.at<double>(rowIdxs[r], colIdxs[r])).copyTo(stokesMatrix.row(r));

                    getIntensityCoeff(_defaultOrientations[r], 0.5, 1.0).copyTo(idealMatrix.row(r));
                }

                // Pseudo inverse has a size of 3 x 4.
                // calibMatrix changes from intensities to intensities, so it is 4 x 4.
                cv::Mat pseudoInverse;
                computePseudoInverse(stokesMatrix, true, pseudoInverse);
                cv::Mat calibMatrix = idealMatrix * pseudoInverse;

                assert((calibMatrix.cols == 4) && (calibMatrix.rows) == 4);
                for (int r = 0; r < calibMatrix.cols; r++)
                {
                    cv::Mat colR = calibMatrix.col(r);
                    _calibrationMatrix[r].at<double>(i, j) = colR.at<double>(0, 0);
                    _calibrationMatrix[r].at<double>(i, j + 1) = colR.at<double>(1, 0);
                    _calibrationMatrix[r].at<double>(i + 1, j) = colR.at<double>(2, 0);
                    _calibrationMatrix[r].at<double>(i + 1, j + 1) = colR.at<double>(3, 0);
                }
            }
        }

        std::cout << "Calibrated Stokes matrices computed" << std::endl;
    }
    else
    {
        std::cout << "ERROR: Calibrated Stokes matrices not computed. The phase matrix is empty" << std::endl;
    }
}
