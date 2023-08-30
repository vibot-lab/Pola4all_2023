#ifndef __SOFTWARE_WHITE_BALANCE_HPP__
#define __SOFTWARE_WHITE_BALANCE_HPP__

// OpenCV includes
#include <opencv2/core.hpp>

// STD includes
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes

/**
 * @brief SoftwareWhiteBalance: Custom implementation of a white balance module.
 * It will compute the corresponding gains for the red, the green and the blue
 * pixels. It takes one image, it converts it into HSV space, and we look for
 * the pixel with the maximum value in this space. Then, we compute the gains
 * in such a way that each channel value in the RGB space have this maximum value.
 *
 * If the image is too dark, we multiply all the pixels by a gain in such a way
 * they get brighter, and then we apply the white balance algorithm.
*/
class SoftwareWhiteBalance
{
public:
    /**
     * @brief Constructor
    */
    SoftwareWhiteBalance();
    ~SoftwareWhiteBalance();

    /**
     * @brief applyBalance: If a color image is provided (3 channels), we apply the
     *  white balance correction, with the gains computed previously. This
     *  application is just a multiplication of each pixel color by its gains
     *  (red pixel value with the red gain, green pixel by green gain, and blue
     *  pixel with blue). If the image is single channel, we do not do anything
     *  to the image.
     *
     * @arg input: Input color image.
     *
     * @returns: The image with the white balance correction applied.
    */
    cv::Mat applyBalance(const cv::Mat &input);

    /**
     * @brief applyBalance: Apply the white balance correction to a set of images.
     *  This function will iterate over the elements of the input vector,
     *  and it will apply the white balance correction function for a single image.
     *
     * @arg input: Vector of color images.
     *
     * @returns: Vector in which the white balance correction has been applied
     *  to each input image.
    */
    std::vector<cv::Mat> applyBalance(const std::vector<cv::Mat> &input);

    /**
     * @brief setRectOfInterest: Set a particular region of interest in which
     *  we will search feature points for computing the white balance correction gains.
     *  This bounding box will be used only if the Auto White Balance gain feature
     *  is enabled. For now, it has not been implemented.
     *
     * @arg topLeftCorner: Vector with the (row, column) coordinates of the top
     *  left corner of the bounding box of interest.
     *
     * @arg bottomRightCorner: Vector with the (row, column) coordinates of the bottom
     *  right corner of the bounding box of interest.
    */
    void setRectOfInterest(std::vector<int> topLeftCorner, std::vector<int> bottomRightCorner);

    /**
     * @brief setGains: Set the internal gains from the exterior of the module.
     *  If the module is in Auto White Balance mode, we do not set them.
     *
     * @arg gains: Vector of gains.
    */
    void setGains(std::vector<double> gains);

    /**
     * @brief getGains: Get a copy of the last set of gains (either computed
     *  automatically, or set by the user).
     *
     * @returns: Vector with 3 elements: Red gain, Green Gain, Blue gain.
    */
    std::vector<double> getGains() const {return _gains;}

    /**
     * @brief isAutoWhiteBalanceEnabled: Check the status of the Auto white
     *  balance feature.
     *
     * @returns: True if the auto white balance feature is enabled.
    */
    bool isAutoWhiteBalanceEnabled() const {return _isAutoWhiteBalanceEnabled;}

    /**
     * @brief enableAutoWhiteBalance: Change the auto white balance feature status.
     *
     * @arg enabled: If true, the auto white balanace feature will be enabled.
    */
    void enableAutoWhiteBalance(bool enabled);

private:
    /**
     * @brief improvedAutoUpdateGains: Update the white balance gain based on an image.
     *  This function will extract the channels of the RGB image and make their average.
     *  Then, it will search the position of the maximum average value, and it will
     *  compute the gains as the division between the maximum color at that position, and
     *  pixel value at that position. The resulting gains will be stored in local variables.
     *
     *  The auto white-balance algorithm works when there are not saturated pixels
     *  in the image.
     *
     * @arg input: Color input image taken as a reference for the white
     *  balance correction.
    */
    void improvedAutoUpdateGains(const cv::Mat &input);

    bool _isAutoWhiteBalanceEnabled;
    std::vector<double> _gains;
    std::vector<int> _rectOfInterest;
    int _valueThreshold;
};

#endif // __SOFTWARE_WHITE_BALANCE_HPP__