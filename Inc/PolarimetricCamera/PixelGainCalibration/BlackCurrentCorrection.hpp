#ifndef __BLACK_CURRENT_CORRECTION_HPP__
#define __BLACK_CURRENT_CORRECTION_HPP__

// OpenCV includes
#include <opencv2/core/core.hpp>

// STD includes
#include <string>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes

// ROS includes

/**
 * @brief BlackCurrentCorrection: This class will be in charge of computing the
 * calibration matrix for the Black current offset. This offset appears due to
 * the sensor is not ideal, and it cannot reach the zero value, when there is
 * no incoming light.
 *
 *  Furthermore, this non-zero value is not equal for each pixel, but it changes
 * from pixel to pixel, and it depends on the temperature, and the exposure time.
 *
 *  This class is aimed to analyze the evolution of this black current with
 * the changes in the exposure time. It will take several samples, with no
 * incoming light, at different exposure times, and it will compute the coefficients
 * of a function that fits to this evolution, pixel-wise. Then, we can extrapolate
 * and get the black current image, for an unknown exposure time, without the need
 * of taking an image on a dark environment.
 *
 *  This procedure is part of the flat-field calibration, and it corresponds to
 * the shift of the pixel value.
*/
class BlackCurrentCorrection
{
public:
    /**
     * @brief Constructor
    */
    BlackCurrentCorrection();
    ~BlackCurrentCorrection() = default;

    /**
     * @brief loadData: It stores internally the results of the experiment.
     *
     * @arg expTimes: Vector with the exposure times at which we have taken
     * each samples
     * @arg avgImgs: Multi-channel image with the averaged samples taken at each
     *  exposure time value.
    */
    void loadData(const std::vector<double>& expTimes, const cv::Mat& avgImgs);

    /**
     * @brief correctImage: Apply the black current correction. This operation
     * consists of making the difference between the calibration matrix and the
     * input image. If the calibration has not been computed yet, it returns the
     * input image
     *
     * @arg rawImg: Input image to correct. It should be an image taken directly
     * from the camera, without any processing done.
     *
     * @arg output: Corrected image
    */
    void correctImage(cv::Mat& rawImg, cv::Mat &output);

    /**
     * @brief getExposuresVector: Get the vector with the different exposure
     * times at which we have taken the samples.
    */
    const std::vector<double>& getExposuresVector() const {return _exposuresVector; }

    /**
     * @brief getSamples: Get the image with the different intensity samples
     *  measurements.
    */
    const cv::Mat& getSamples() const {return _sampleImgs; }

    /**
     * @brief getOffsetsMatrix: Get a copy of the calibration matrix.
    */
    const cv::Mat& getOffsetsMatrix() const {return _calibrationMatrix; }

    /**
     * @brief calibrate: Compute the calibration matrix. Optionally, we can store
     * the calibration matrix once computed.
     *
     * @arg outputFilename: Filepath where we want to store the calibration matrix.
     * If this string is empty, it does not store the matrix.
    */
    void calibrate(std::string outputFilename);

    /// \brief isDataLoaded: Check if the experiment data has been loaded.
    bool isDataLoaded() const { return !_sampleImgs.empty(); }

    /// \brief areCalibrationComputed: Check if the calibration matrix has been computed.
    bool areCalibrationComputed() const { return !_calibrationMatrix.empty();}

private:
    /// \brief reset: Clear all the internal buffers
    void reset();

    cv::Mat _calibrationMatrix;
    std::vector<double> _exposuresVector;
    cv::Mat _sampleImgs;
};
#endif // __BLACK_CURRENT_CORRECTION_HPP__