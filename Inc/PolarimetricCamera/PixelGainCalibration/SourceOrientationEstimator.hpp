#ifndef __SOURCE_ORIENTATION_ESTIMATOR_HPP__
#define __SOURCE_ORIENTATION_ESTIMATOR_HPP__

// OpenCV includes
#include <opencv2/core/core.hpp>

// STD includes
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes

// ROS includes

/**
 * @brief SourceOrientationEstimator: This module serves to estimate the orientation
 * of a linear polarized light. It considers the input images divided in
 * blocks of four elements, arranged in a matrix of 2 x 2 elements. This patterns
 * are repeated over all the image, in both directions, horizontal and vertical.
 * These blocks are considered to be like this:
 *                              || I135 | I0  ||
 *                              || I90  | I45 ||
 * where {I135, I0, I90, I45} are the intensities corresponding to the pixels with
 * microgrid orientations of 135, 0, 90 and 45 degrees, respectively.
 *
 * These values can be changed on construction of the class.
 * If the camera is estimulated by an uniform linearly polarized light, all
 * the pixels should receive the same intensity measurement. Due to the lens,
 * a deformation of the state of the light occurs for all the points in which
 * the light does not arrive parallel to the lens axis. Thus, only a small region
 * around the center is considered. Each block of 2 x 2 in this region can
 * detect the orientation of the source light. Then, it is possible
 * to retrieve a good estimation of the orientation of linearly polarized light
 * by doing the average of all the detected orientations in the ROI.
*/
class SourceOrientationEstimator
{
public:
    /**
     * @brief Constructor.
     *
     * @arg camFilterOrient: Vector with the mapping indexes of the orientations order.
     *  Taking a block of 2 x 2 elements, the order of the pixels is:
     * (0,0), (0, 1), (1, 0), (1,1). Then, each element of this vector represents the index
     * of the corresponding orientation within the block:
     *  - The first element is the index of the measurement at 135.
     *  - The second element is the index of the measurement at 0.
     *  - The third element is the index of the measurement at 90.
     *  - The forth element is the index of the measurement at 45.
     *
     *  Examples: the vector [0, 1, 2, 3] means the detector block is ordered
     * as this: {I135, I0, I90, I45}
     *  The vector [3, 0, 2, 1] means the detector block is ordered
     * as this: {I0, I45, I90, I135}
     * @arg roiSize: region of interest size. The ROI will be a square zone around
     *   the center. It is measured in super-pixels. Note that only even numbers
     *   should be used.
    */
    SourceOrientationEstimator(std::vector<int> camFilterOrient, int roiSize);
    ~SourceOrientationEstimator() = default;

    /**
     * @brief getLightSourceOrientation: Estimate the angle of polarization
     * of the light source. It will take pixels in groups of 2 x 2 elements, and
     * it will estimate the orientation of the light they receive. This procedure
     * is done only with some pixels around the center of the sensor.
     * In order to compute the orientation of the light, we use the pseudo inverse method.
     *  If we consider a set of 4 measurements of the light, with a polarization
     * analyzer composed by 4 pixels with different orientations {O1, O2, O3, O4},
     * and an ideal camera, we can set a matricial equation as:
     *
     *                           I = C * S
     * where:
     *      I = [I1 I2 I3 I4]^T is the set of measurements arranged as a column vector
     *      S = [K * A  K * cos(2*alpha) K * sin(2*alpha)] is the stokes vector
     *      K = SO * dop / 2 is a constant.
     *      A = 1 / dop is a constant
     *      alpha is the orientation we are looking for.
     *          | 1    cos (2*O1)    sin (2*O1) |
     *      C = | 1    cos (2*O2)    sin (2*O2) |
     *          | 1    cos (2*O3)    sin (2*O3) |
     *          | 1    cos (2*O4)    sin (2*O4) |
     *
     * Then, we can get a solution by computing the pseudo inverse of C, called C^+
     *
     *                         C^+ * I = S
     *  If we write the solution S = [s1 s2 s3] = [K * A  K * cos(2*alpha) K * sin(2*alpha)],
     * then, the orientation we are looking for is:
     *                  alpha = 0.5 * atan2(s3, s2)
     *
     *  If we consider the default orientations of the micro-filters:
     * (135, 0, 90, 45) degrees, the pseudo inverse is fixed, and it has the
     * following shape:
     *                    |  0.5     0.5    0.5   0.5 |
     *              C^+ = |  0.      1.    -1.    0.  |
     *                    | -1.      0.     0.    1.  |
     *
     *  Therefore, we can directly compute S1 as I0 - S90 and S2 as I45 - I135.
     *  Then, we compute the circular average of all the detected angles.
     *
     * @arg input: Input image. It should correspond to the image of an uniform,
     *  linearly polarized light.
     *
     * @returns: Average detected orientation of the linearly polarized light
     *  received by the camera.
    */
    double getLightSourceOrientation(const cv::Mat &input) const;

    /**
     * @brief updateSPOrientations: Update the super-pixel filter
     *   orientations order.
     *
     * @arg newOrientations: Vector with the new indexes, with respect
     *   to the order [135, 0, 90, 45], as explained in the constructor
     *   function.
    */
    void updateSPOrientations(std::vector<int> newOrientations);

private:
    cv::Mat _pseudoInverse;

    std::vector<int> _I135Offset;
    std::vector<int> _I0Offset;
    std::vector<int> _I90Offset;
    std::vector<int> _I45Offset;

    int _centerHeight;
    int _centerWidth;
};
#endif // __SOURCE_ORIENTATION_ESTIMATOR_HPP__