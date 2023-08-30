#ifndef __SUPER_PIXEL_CALIBRATION_HPP__
#define __SUPER_PIXEL_CALIBRATION_HPP__

// OpenCV includes
#include <opencv2/core/core.hpp>

// STD includes
#include <map>
#include <string>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes
#include "CameraTypes.hpp"

// ROS includes

/**
 * @brief SuperPixelCalibration class: This class produces the pixel gain
 * calibration matrix.
 * For a fixed, homogeneous light, when all the pixels are exposed, we should
 * get the same measurements for the pixels with the same color filter,
 * and the same polarization filter. For instance, all the red pixels, with a
 * linear polarizer oriented at 0 degrees must have the same measurement value.
 *
 *   Generally, this is not the case. Each pixel contains a different response, so
 * their measurements will differ. On the other hand, we know the camera pixels are
 * arranged in groups of 4 different linear polizer filters: 0, 45, 90 and 135
 * degrees. These angles are not real either. The polarizers are placed with
 * certain error, and there are not two pixels with exactly the same orientation.
 *
 *   As a consequence, to take measurements with this camera, we need to know
 * these parameters, with respect to the same reference.
 *
 *   The required  experiment to do this calibration (gain and polarizer filters)
 * is to expose the pixels to a single, unpolarized light, and place a linear
 * polarization filter in front of it. This filter will let pass only the components
 * with a specific polarization orientation. If we turn this polarizer for several
 * angles, between 0 and 360 degrees (actually, with the interval 0 - 180 degrees
 * will be enough), the different pixel values will describe a shifted cosine
 * function.
 *
 *   Each pixel will follow a function like this:
 *          Iij = Ti * S0 / Pi +
 *                Ti * S0 * dop * cos(2 * alpha_j) * cos(2 * theta_i) +
 *                Ti * S0 * dop * sin(2 * alpha_j) * sin(2 * theta_i)
 *  where:
 *      Ti is the transmission coefficient (generally 0.5).
 *      S0 is the intensity of the source light
 *      dop is the degree of polarization of the source light
 *      Pi modelates the non ideality of the pixel
 *      alpha_j is the orientation of the polarization of the source light for the sample j
 *      theta_i is the orientation of the micro-polarizer of the pixel
 *
 *  Doing some mathematical manipulations, we obtain this equation:
 *          Iij = A + B * cos(2* (alpha_j - theta_i))
 *  where:
 *      A = Ti * S0 / Pi
 *      B = Ti * S0 * dop
 *
 *  Therefore, any solution implemented will find the values of A and B, and then,
 * we can compute the values of Ti, S0, Pi, and dop.
 *
 *   Nonetheless, it is not possible to find all the parameters from these two
 * equations. We need the knowledge of S0 and dop, in order to compute Ti and Pi.
 * In other words, for each pair (S0, dop), we will obtain a pair (Ti, Pi), which
 * means that our solution is relative the values S0 and dop used during calibration.
 *
 *   This class is inspired in the concept on the paper
 * "Calibration methods for division-of-focal-plane polarimeters" by S. B. Powell
 * and V. Gruev. Each set of parameters is computed considering the super-pixel
 * values, i.e., we take four contiguous measurements of the same color,
 * and we compute their parameters together, in a single step.
 *
 *   The authors use a complex arrangement, from which they know exactly the
 * Stokes vector of the samples. In our case, we do not know the values of S0
 * and dop. We only know the orientation of the angles of the samples.
 * To determine the values of S0 and dop, we assume our camera
 * is not that bad, we compute A and B, and we suppose the pixels as ideal. This
 * allows us to obtain a measured value of S0 and dop per pixel. Finally, we can
 * choose to take the average, the median or the maximum value to estimate their
 * values, and we consider them as our true values to compute Ti and Pi for each pixel.
 *
 *  Additionally, since the mathematical problem is the same, we implement a
 * single pixel version (it is not the same as the single-pixel method of Powell and Gruev).
 * This method uses several samples of a single pixel to find its parameters during calibration
 * (gain, non-ideality, and filter orientation). To choose one method or the other
 * one, we change the boolean value when we call the function computeParametersMatrices.
*/
class SuperPixelCalibration
{
public:
    /**
     * @brief Constructor
    */
    SuperPixelCalibration();
    ~SuperPixelCalibration() = default;

    /**
     * @brief loadData: It retrieves the different angles and averaged images
     *  from the samples, and it stores them locally.
     *
     * @arg angles: Vector with the polarization angles at which each sample has been taken.
     * @arg avgImgs: Multi-channel image where each channel is the average images
     *  taken at the corresponding polarization angle.
    */
    void loadData(const std::vector<double>& angles, const cv::Mat& avgImgs);

    /**
     * @brief getPhaseShiftMatrix: Get a copy of the image that contains the pixel
     * filter orientation, in degrees, after calibration. This image contains
     * values between 0 and 180.
    */
    cv::Mat getPhaseShiftMatrix() const {return _phasesImage.clone();}

    /**
     * @brief getTiImage: Get a copy of the image that contains the Ti parameter
     * of the cosine function, after curve fitting.
    */
    cv::Mat getTiImage() const {return _TiImage.clone();}

    /**
     * @brief getPiImage: Get a copy of the image that contains the Pi parameter
     * of the cosine function, after curve fitting.
    */
    cv::Mat getPiImage() const {return _PiImage.clone();}

    /// \brief getCalibrationDoP: Get the degree of polarization of the light used during calibration
    std::vector<double> getCalibrationDoP() const { return _lightDoP; }

    /// \brief getCalibrationS0: Get the intensity of the light used during calibration
    std::vector<double> getCalibrationS0() const { return _lightS0; }

    /**
     * @brief getAnglesVector: Get a vector with all the measurement angles from
     * the calibration experiment.
    */
    const std::vector<double>& getAnglesVector() const {return _anglesVector;}

    /**
     * @brief getSamples: Get an image with all the sampled intensities retrieved
     * from the calibration experiment.
    */
    const cv::Mat& getSamples() const {return _allImagesSamples;}

    /**
     * @brief computeParametersMatrices: Generate the matrices of the different
     * pixel parameters. These matrices are related
     * to the experiment: if we take N samples of a linearly polarized light that changes
     * the angle of linear polarization in the interval 0 - 180 degrees, we should
     * obtain a complete cycle of a shifted cosine function.
     *  The corresponding equation of this cosine function is:
     *
     *            Iij = Ti * S0 / Pi +
     *                  Ti * S0 * dop * cos(2 * alpha_j) * cos(2 * theta_i) +
     *                  Ti * S0 * dop * sin(2 * alpha_j) * sin(2 * theta_i)
     *       -->  Iij = (S0 * Ti / Pi) + (Ti * S0 * dop) * cos(2 * (alpha_j - theta_i)))
     *       -->  Iij = A + B * cos(2 * (alpha_j - theta_i))
     *
     *  where:
     *      Ti is the transmission coefficient (generally 0.5).
     *      S0 is the intensity of the source light
     *      dop is the degree of polarization of the source light
     *      Pi models the non ideality of the pixel (ideally 1.0)
     *      alpha_j is the AoLP of the input source light for the sample j
     *      theta_i is the orientation of the micro-polarizer of the pixel
     *
     *   If we take several samples, we can build a linear system of equations like this:
     *
     *                              I = C * L * O
     *  where:
     *         | I135_0      I135_j    I135_N |
     *   I =   | I0_0   ...  I0_j  ... I0_N   |
     *         | I90_0       I90_j     I90_N  |
     *         | I45_0       I45_j     I45_N  |
     *
     *         | T_135 / P_135    T_135 * cos (2 * theta_135)  T_135 * sin (2 * theta_135) |
     *   C =   | T_0 / P_0        T_0 * cos (2 * theta_0)      T_0 * sin (2 * theta_0)     |
     *         | T_90 / P_90      T_90 * cos (2 * theta_90)    T_90 * sin (2 * theta_90)   |
     *         | T_45 / P_45      T_45 * cos (2 * theta_45)    T_45 * sin (2 * theta_45)   |
     *
     *            | 1      0      0     |
     *   L = S0 * | 0     dop     0     |
     *            | 0      0     dop    |
     *
     *         |        1          ...        1          ...        1          |
     *   O =   | cos (2 * alpha_0) ... cos (2 * alpha_j) ... cos (2 * alpha_N) |
     *         | sin (2 * alpha_0) ... sin (2 * alpha_j) ... sin (2 * alpha_N) |
     *
     *  Finally, we can do the pseudo-inverse of O, and solve the equation as:
     *
     *                      C * L = I * B
     * where B is the pseudo inverse of O : B = O^T * (O * O^T)^-1
     *
     * Thus, we will have an equation that depends on the values of S0, Ti, Pi,
     * dop and phaseShift for each pixel.
     * Note that we only know the intensity measurements, and the orientation of the angles
     * of the samples. To determine the values of S0 and dop, we assume our camera
     * is not that bad, we compute A and B, and we assume several central pixels as ideal. This
     * allows us to obtain a measured value of S0 and dop per pixel. Then, we estimate
     * the light parameters by doing either the average, the maximum value or the median value
     * of these measurements. Finally, we use these parameters as the true values to compute
     * Ti and Pi for each pixel.
     *
     *   This function can be modulated to compute the Single-pixel version
     * of this algorithm. The only thing that changes is the size of the intensity
     * matrix: instead of having a size of 4 x N, it will have a size of 1 x N.
     * The pseudo inverse is N x 3, so the final vector will have a size of 1 x 3,
     * which corresponds to the row vector:
     *      [ T_i / P_i    T_i * cos (2 * theta_i)  T_i * sin (2 * theta_i) ]
     *
     * @arg useSuperPixel: Boolean. If true, we compute the Super pixel calibration.
     *  If false, we compute the Single pixel calibration.
    */
    void computeParametersMatrices(bool useSuperPixel);

    /// \brief isDataLoaded: Check if the experiment data has been loaded.
    bool isDataLoaded() const;

    /// \brief areCalibrationComputed: Check if the calibration matrices have been computed.
    bool areCalibrationComputed() const;

    /**
     * @brief correctImage: Correct the raw image intensities using the calibration results.
     *   To execute this function, the load data and calibration steps should have been run before.
     *  If this has not been done previously, the input raw image is returned.
     *
     * @arg rawImg: Input raw image from the camera.
     * @arg output: Output image.
    */
    void correctImage(cv::Mat& rawImg, cv::Mat &output);

    /**
     * @brief updateDefaultFilterOrientations: Update the default filter orientations.
     *   This function will reset the calibration Stokes matrix, and if there is calibration
     *  data already loaded, it will recompute the calibration matrices based on this new orientations.
     *
     * @arg newOrientations: Vector with the new filter orientations order. They must be given
     *   in degrees, and the vector must contain 4 elements.
    */
    void updateFilterOrientations(std::vector<int>& newOrientations);

private:
    /// \brief reset: Clear all the internal buffers
    void reset();

    /**
     * @brief getColorIndex: Determine if certain pixel has a red, green or blue
     * color filter.
     *
     * @arg row: Row of the pixel to evaluate.
     * @arg col: Column of the pixel to evaluate.
     *
     * @returns: Index that represents the color: 0 means red, 1 means green and
     * 2 means blue.
    */
    int getColorIndex(int row, int col);

    /**
     * @brief optimize: Find a set of parameters for one or four pixels, as
     *  explained for the function computeParametersMatrices.
     *
     * @arg sampPseudoInv: Pseudo inverse of the matrix with the cosine and sine
     *  functions of the sample's orientation
     * @arg samples: Observations taken with the camera for a super pixel (shape = 4 x N)
     * @arg optimizedParams [output]: Struct with the pixels parameters found.
    */
    void optimize(const cv::Mat& stokesPseudoInv,
        const cv::Mat& samples,
        SuperPixelParams& cameraParams);

    /**
     * @brief saveCalibrationImage: Save the calibration matrices into YAML files.
     *  This function uses the OpenCV FileStorage class, since we cannot store
     *  images of type CV_64F.
     *
     * @arg img: Image to be written in disk.
     * @arg filepath: Absolute path of the folder where we want to store the calibration
     *  matrix. The information will be stored in the field called "mat1".
    */
    void saveCalibrationImage(cv::Mat& img, std::string filepath);

    /**
     * @brief initializeMatricesSizes: Place the default values into the internal
     *  matrices, before running the calibration algorithms.
     *
     * @arg rows: Amount of rows of the input images.
     *
     * @arg cols: Amount of columns of the input images.
    */
    void initializeMatricesSizes(int rows, int cols);

    /**
     * @brief computeSamplesPseudoInverse: Create the pseudo-inverse of the samples
     * AoLP matrix. The original matrix has the following shape:
     *
     *         |        1          ...        1          ...        1          |
     *   O =   | cos (2 * alpha_0) ... cos (2 * alpha_j) ... cos (2 * alpha_N) |
     *         | sin (2 * alpha_0) ... sin (2 * alpha_j) ... sin (2 * alpha_N) |
     *
     *  The output is the pseudo inverse of O defined as O^T * (O * O^T)^-1, where
     *  (.)^T indicates the transpose operation.
     *  (.)^-1 indicates the inverse operation.
     *  (.) * (.) indicates the matrix multiplication operation.
     *
     * @arg angles: Values to be used as alpha_j. The amount of columns of the
     *  original matrix is the same as the amount of elements in this vector.
     *
     * @returns: cv::Mat with the pseudo-inverse matrix of O.
    */
    cv::Mat computeSamplesPseudoInverse(std::vector<double>& angles);

    /**
     * @brief addLightParamsSample: Add samples to compute the light parameters.
     * The resulting values will be stored (at the end), in _lightS0 and _lightDoP.
     *  For the average method, we do the running average in place, and for the
     * method of maximum value, we find the maximum found value between the samples,
     * and we store it directly. In the case of the median method, this function
     * only stores the detected S0 and DoLP values. When all the pixels have
     * been considered, the median value must be computed and stored in _lightS0
     * and _lightDoP. This is done by the function computeLightParamsMedian.
     *
     * @arg TiPiS0: Factor that involves the factor Ti * S0 / Pi
     * @arg TiS0DOP: Factor that involves the factor Ti * S0 * dop
     * @arg colorIdx: Number (0, 1 or 2) that identifies the color filter that
     *  the involved pixel has.
     * @arg method: Estimation method (METHOD_AVERAGE, METHOD_MAXIMUM, METHOD_MEDIAN)
    */
    void addLightParamsSample(double TiPiS0, double TiS0DOP, int colorIdx, LightEstimationMethod method);

    /**
     * @brief computeLightParamsMedian: Compute the median values of the light
     *  source parameters. This function must be executed after finishing the
     *  pixel parameters estimatation, when the METHOD_MEDIAN has been selected.
     *  It will take all the values stored in _lightAllValues vector, and find
     *  the mean values for each color, separatedly. It will also store the
     *  final values in the variables _lightS0 and _lightDoP.
    */
    void computeLightParamsMedian();

    /**
     * @brief computeCorrectionMatrix: Compute the Stokes matrix for the corrected
     * orientation values. The computed matrix is a vector of matrices.
     * Each image corresponds to the set of coefficients that we
     * have to multiply each intensity measurement in order to obtain the corrected
     * intensity.
     *
     *  The first image corresponds to all the coefficients with which we have
     * to multiply the first filter intensities, the second image corresponds
     * to the coefficients with which we have to multiply the second filter
     * intensity, and so on.
     *
     *  Then, we can compute the corrected intensities by doing the linear combination
     * between the intensity samples, and the corresponding correction coefficients.
    */
    void computeCorrectionMatrix();

    /**
     * @brief computePseudoInverse: Function to compute the left / right
     *   pseudo inverse of a matrix.
     *
     * @arg matrix: Input matrix.
     * @arg isLeft: If true. we compute the left side pseudo-inverse (A^+ * X).
     *   If false, the right side pseudo-inverse is computed (X * A^+).
    */
    inline void computePseudoInverse(cv::Mat matrix, bool isLeft, cv::Mat& output)
    {
        if (isLeft)
        {
            output = (matrix.t() * matrix).inv() * matrix.t();
        }
        else
        {
            output = matrix.t() * (matrix * matrix.t()).inv();
        }
    }

    /**
     * @brief getIntensityCoeff: Compute the coefficients that multiply the
     *      stokes vector to get the corresponding intensity when the polarization
     *      angle is alpha and the non-ideality parameters are Ti and Pi.
     *
     *      I_alpha = C_1 * S0 + C_2 * S1 + C_3 * S2
     *      C_1 = Ti / Pi
     *      c_2 = Ti * cos (2 * alpha)
     *      c_3 = Ti * sin (2 * alpha)
     *
     * @arg alpha: Angle of linear micro-polarizer filter at the pixel level.
     * @arg Ti: Non ideality parameter. The polarizer filter absorbs part of the light.
     * @arg Pi: Non ideality parameter. The polarizer does not follow exactly
     *  Malus' law.
     *
     * @return cv::Mat with 1 row, and 3 columns, with the coeficients
     *      {C_1, C_2, C_3}
    */
    inline cv::Mat getIntensityCoeff(float theta, double Ti, double Pi) const
    {
        theta = theta * M_PI / 180.0;
        double data[3] = {Ti / Pi, Ti * std::cos(2.0*theta), Ti * std::sin(2.0*theta)};
        return cv::Mat(1, 3, CV_64FC1, data).clone();
    }

    cv::Mat _allImagesSamples;
    std::vector<double> _anglesVector;

    cv::Mat _phasesImage;
    cv::Mat _TiImage;
    cv::Mat _PiImage;

    std::vector<double> _lightDoP;
    std::vector<double> _lightS0;
    std::vector<std::vector<std::pair<double, double>>> _lightAllValues;
    std::vector<int> _N;
    std::vector<int> _roiHW;

    std::map<std::pair<int, int>, uint8_t> _colorLabels;
    std::vector<cv::Mat> _calibrationMatrix;
    std::vector<int> _defaultOrientations;
};

#endif // __SUPER_PIXEL_CALIBRATION_HPP__
