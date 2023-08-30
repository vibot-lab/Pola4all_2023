#ifndef __CALIBRATORS_HUB_HPP__
#define __CALIBRATORS_HUB_HPP__

// OpenCV includes
#include <opencv2/core/core.hpp>

// STD includes
#include <string>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes
#include "PolarimetricCamera/PixelGainCalibration/BlackCurrentCorrection.hpp"
#include "PolarimetricCamera/PixelGainCalibration/SuperPixelCalibration.hpp"

// ROS includes

/**
 * @brief CalibratorsHub: This class serves to apply the flat-field calibration.
 * It makes use of the black current and the pixel gain calibration modules.
 *
 *  Since the samples grabbing procedure will be the same for both calibrators,
 * this class provides a vector with the calibrator names, and then the user has to provide
 * a string with the name of the calibrator to which it wants to give the data.
 *
 *  It incorporates some getters to retrieve the different calibration matrices,
 * to do further analysis of the results.
*/
class CalibratorsHub
{
public:
    /**
     * @brief: Constructor
    */
    CalibratorsHub();
    ~CalibratorsHub() = default;

    /// \brief getListCalibrators: Get a vector with the names of the different calibrators integrated
    std::vector<std::string> getListCalibrators() const { return _calibratorsNames; }

    /**
     * @brief getCalibratorIndex: Convert the calibrator name into an ID.
     * This will ease the processus of detecting which calibrator we have to
     * use. If the provided name is not correct, it will throw an assertion
     * exception.
     *
     * @arg selectedCalib: Name of the selected calibrator.
    */
    int getCalibratorIndex(std::string selectedCalib) const;

    /// \brief getPhaseShiftMatrix: Get a copy of the matrix with the cosine function phase shift of each pixel
    cv::Mat getPhaseShiftMatrix() const { return _gainCalibrator.getPhaseShiftMatrix(); }

    /// \brief getBlackCurrentMatrix: Get a copy of the black current offset matrix
    cv::Mat getBlackCurrentMatrix() const { return _offsetCalibrator.getOffsetsMatrix(); }

    /// \brief getTiImage: Get a copy of the matrix with the Ti parameter of each pixel
    cv::Mat getTiImage() const { return _gainCalibrator.getTiImage(); }

    /// \brief getPiImage: Get a copy of the matrix with the Pi parameter of each pixel
    cv::Mat getPiImage() const { return _gainCalibrator.getPiImage(); }

    /// \brief getCalibrationDoP: Get the degree of polarization of the light used during calibration
    std::vector<double> getCalibrationDoP() const { return _gainCalibrator.getCalibrationDoP(); }

    /// \brief getCalibrationS0: Get the intensity of the light used during calibration
    std::vector<double> getCalibrationS0() const { return _gainCalibrator.getCalibrationS0(); }

    /// \brief getAmplitudesMatrix: Get the calibrator name based on its ID
    std::string getCalibratorName(Calibrators id) const;

    /// \brief isDataLoaded: Check if the experiment data has been loaded in the specified calibrator
    bool isDataLoaded(std::string selectedCalib) const;

    /// \brief areCalibrationComputed: Check if the calibration matrices have been computed in the specified calibrator
    bool areCalibrationComputed(std::string selectedCalib) const;

    /**
     * @brief loadData: Store the experiment samples in the specified calibrator.
     *
     * @arg selectedCalib: Name of the calibration to which we are going to provide
     * this data.
     * @arg camFilterOrient: Vector with the mapping indexes of the orientations order
     *  of the 2x2 super-pixels in the camera. These orientations are read from left to
     *  right, and from top to bottom.
     *  - The first element is the index of the measurement at 135.
     *  - The second element is the index of the measurement at 0.
     *  - The third element is the index of the measurement at 90.
     *  - The forth element is the index of the measurement at 45.
     *
     *  Example: the vector [0, 1, 2, 3] means the detector block is ordered
     * as this: {I135, I0, I90, I45}
     *  The vector [3, 0, 2, 1] means the detector block is ordered
     * as this: {I0, I45, I90, I135}
     * @arg horAxis: Vector with the horizontal axis values at which each sample has been taken.
     * For instance, if the selected calibrator is the pixel gain, the horizontal axis
     * corresponds to the different polarization angles. If the selected calibrator
     * is the black current, the horizontal axis vector contains the exposure times.
     * @arg avgImgs: Multi-channel image where each chanells is the average
     * image at the corresponding value of the horizontal axis vector.
    */
    void loadData(std::string selectedCalib,
        std::vector<int> camFilterOrient,
        const std::vector<double>& horAxis,
        const cv::Mat& avgImgs);

    /// \brief getHorizontalAxis: Get a copy of the values at which each sample has been taken
    std::vector<double> getHorizontalAxis(std::string selectedCalib) const;

    /// \brief getRawSamples: Get an image with a light sample at each channel.
    const cv::Mat& getRawSamples(std::string selectedCalib) const;

    /**
     * @brief computeCalibrationMatrices: Compute the calibration matrices for all the calibrators.
     *
     * @arg useSpCalib: If true, we use the algorithm of the super pixel calibration,
     * if not, we use the single-pixel.
     */
    void computeCalibrationMatrices(bool useSpCalib);

    /**
     * @brief correctImage: Apply the flat-field calibration. This operation
     * will apply first the offset correction, and then, the gain calibration.
     *  If any of the calibrators has not been initialized, it will return
     * the image without applying its correction.
     *
     * @arg rawImg: Input image to correct. It should be an image taken directly
     * from the camera, without any processing done.
     *
     * @arg correct: Boolean. If true, we apply the correction to the input image.
     *  If false, it returns the raw image given as input.
     *
     * @arg output: Corrected image
     */
    void correctImage(cv::Mat rawImg, bool correct, cv::Mat &output);

    /**
     * @brief updateDefaultFilterOrientations: Update the default filter orientations.
     *   This function will reset the calibration Stokes matrix, and if there is calibration
     *  data already loaded, it will recompute the calibration matrices based on this new orientations.
     *
     * @arg newOrientations: Vector with the new filter orientations order. They must be given
     *   in degrees, and the vector must contain 4 elements.
    */
    void updateDefaultFilterOrientations(std::vector<int>& newOrientations)
    {
        _gainCalibrator.updateFilterOrientations(newOrientations);
    }

    /**
     * @brief getGainCalibName: Get the string that corresponds to the pixel
     *  gain calibrator.
     */
    std::string getGainCalibName() const {return _calibratorsNames[PIXEL_GAIN];}

    /**
     * @brief getOffsetCalibName: Get the string that corresponds to the offset
     *  calibrator.
     */
    std::string getOffsetCalibName() const {return _calibratorsNames[BLACK_CURRENT];}

private:
    std::vector<std::string> _calibratorsNames;

    SuperPixelCalibration _gainCalibrator;
    BlackCurrentCorrection _offsetCalibrator;
};

#endif // __CALIBRATORS_HUB_HPP__