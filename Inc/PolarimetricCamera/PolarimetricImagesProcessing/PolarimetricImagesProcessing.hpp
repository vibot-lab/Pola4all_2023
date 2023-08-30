#ifndef __POLARIMETRIC_IMAGES_PROCESSING_HPP__
#define __POLARIMETRIC_IMAGES_PROCESSING_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes
#include <map>
#include <vector>

// Qt Includes

// Pylon includes

// Custom includes
#include <CameraTypes.hpp>

/**
 * @brief PolarimetricImagesProcessing class
 *
 *  This class is an static class, that offers several processing functions
 * for the polarimetric camera images. After receiving a raw image from the camera,
 * (corrected or not by a calibration algorithm), we can:
 *   - Split the image into the different polarization angles,
 *   - Convert them into a color image,
 *   - Extract the stokes images,
 *   - Extract the intensity - AOP - DOP images,
 *   - Obtain the original color image,
 *   - Remove specular, polarized reflections,
 *   - Apply software polarization filtering to the image,
 *   - Create the fake color image out of the polarization parameters,
 *
 *  This class assumes the images are in BayerRG format.
*/
class PolarimetricImagesProcessing
{
public:
    PolarimetricImagesProcessing() {}
    ~PolarimetricImagesProcessing() = default;

    /**
     * @brief processImage: Function that produces the desired type of images,
     *      given the desired action.
     *      - action = RAW_IMAGE:
     *          It will return a single element vector, with the retrieved
     *         raw image from the camera
     *      - action = RAW_SPLITTED_IMAGES
     *          It will return a vector with 4 elements. Each element represents one
     *         image with all the pixels with the same polarization angle. Considering
     *         the image as composed by 2x2 super-pixels, the order
     *         of the returned images is:
     *             -) Top left element pixels,
     *             -) Top right element pixels,
     *             -) Bottom left element pixels,
     *             -) Bottom right element pixels.
     *         All the images are single channel images, with the BayerRG color pattern.
     *      - action = COLOR_SPLITTED
     *          Same as RAW_SPLITTED_IMAGES, but each image is a color image,
     *         with 3 channels, in format BGR.
     *      - action = COLOR_ORIGINAL
     *          It will return a single image that is the average of the 4
     *         polarized images returned by RAW_SPLITTED_IMAGES, and demosaicked.
     *      - action = SIM_POLARIZER
     *           Simulate the effect of placing a linear polarizer in front
     *          of a normal camera. Taking the value provided in the aop variable of the
     *          params struct, it computes the intensity image resulting of passing the
     *          input Stokes vector by a Mueller matrix of a linear polarisation filter
     *          oriented at the value in aop. The output vector is composed of two images.
     *          The first one corresponds to the original intensity image, and the second one
     *          to the filtered image. Both images are already in BGR.
     *      - action = FAKE_COLORS
     *           Take the AoLP, DoLP, and S0 images and place them into a
     *          Hue - Saturation - Value image, respectively. Then
     *          we convert it into BGR image to show it. Four fake images are created,
     *          one per color channel (red, green_1, green_2 and blue, respectively).
     *      - action = REMOVE_SPECULARITY
     *           It will take the input image, compute the Stokes
     *          parameters, and with them, compute the AoLP. Then, we apply a linear
     *          polarisation filtering, with an orientation the AoLP + 90 degrees.
     *          It repeats this procedure pixelwise, and with that, we can erase the
     *          specularity in the image. The output vector contains two images: the
     *          original one and the filtered one. Both outputs are already in BGR system.
     *      - action = STOKES
     *          It returns three images. Each image corresponds to one of the Stokes
     *         parameters. Since our camera analyses only the linear polarization,
     *         the images we provide are the ones corresponding to S0, S1 and S2.
     *         Each Stokes parameter is, additionally, split by color channel, since
     *         the demosaicked Stokes vector does not provide any valuable physical information.
     *      - action = I_RO_PHI
     *          It returns the images corresponding to the intensity, the angle
     *         of polarization (AoP), and the degree of linear polarization (DoLP).
     *         The images are separated by color channel. The DoP and AoP are colored
     *         using color palettes (Jet for the DoLP and HSV for the AoLP).
     *      - action = RAW_I_RO_PHI
     *          It returns the intensity, degree of polarization and angle of
     *         polarization, as gray-level images.
     *
     * @arg input: Raw image taken from the Basler camera, in BayerRG format.
     * @arg params: Parameters object.
     *
     * @arg output [output]: Vector with the resulting images. The amount of images
     *      varies based on the processing level:
     *      - RAW_IMAGE: 1 element
     *      - RAW_SPLITTED_IMAGES: 4 element
     *      - COLOR_SPLITTED: 4 element
     *      - COLOR_ORIGINAL: 1 element
     *      - SIM_POLARIZER: 2 elements
     *      - FAKE_COLORS: 4 elements
     *      - REMOVE_SPECULARITY: 2 elements
     *      - STOKES: 12 element
     *      - I_RO_PHI: 12 element
     *      - RAW_I_RO_PHI: 12 elements
    */
    static void processImage(const cv::Mat &input,
        const ProcessingParameters& params,
        std::vector<cv::Mat> &output);

private:
    /**
     * @brief splitImages: Take the raw image from the polarimetric camera, and split it
     *  into 4 images. Each image contains all the pixels corresponding to the same
     *  polarization angle.
     *
     * @arg input: Raw image from the camera, in OpenCV image format.
     * @arg output: Vector in which we return the 4 images retrieved from the input. Each
     *      reconstructed image is a single channel image, in OpenCV image format cv::Mat.
            Considering the image as composed by 2x2 super-pixels, the order of the
            returned vector is:
     *             -) Top left element pixels,
     *             -) Top right element pixels,
     *             -) Bottom left element pixels,
     *             -) Bottom right element pixels.
    */
    static void splitImages(const cv::Mat &input, std::vector<cv::Mat> &output);

    /**
     * @brief demosaickImages: Create the corresponding color images by a normal
     *      Bayer interpolation. This function assumes that input image is
     *      in BayerRG color pattern.
     *
     * @arg input: Reference to the input images. This variable must be a std::vector
     *  with single channel images, that represents the color image filled with BayerRG
     *  patterns.
     * @arg output: Reference to the vector where we will write the demosaicked images.
     *  The output will be as many images as in the input, with 3 channels each,
     * in format BGR.
    */
    static void demosaickImages(std::vector<cv::Mat> &input, std::vector<cv::Mat> &output);

    /**
     * @brief computeStokes: From a raw image taken from the camera, we compute the
     * images of S0, S1 and S2, corresponding to the Stokes model.
     *
     *   This function considers the camera as ideal, thus the computations done are
     *  the following:
     *                  S0 = 0.5 * (I0 + I45 + I90 + I135)
     *                  S1 = I0 - I90
     *                  S2 = I45 - I135
     *
     * @arg input: Raw image taken with the polarimetric camera (corrected by calibration or not).
     * @arg filtersMap: Map from filter orientations to position in the super-pixel array.
     * @arg output: Output vector with the first three stokes parameters as floating point images.
    */
    static void computeStokes(const cv::Mat& input, std::map<int, int> filtersMap, std::vector<cv::Mat> &output);

    /**
     * @brief computeIRoPhi: Convert the Stokes vector images into the images
     * of intensity, degree of linear polarization (DoLP), and angle of polarization
     * (AoP). The intensity image is just the S0 component of the Stokes vector.
     * The DoLP can be computed as:
     *                      DoLP = (sqrt (S1^2 + S2^2)) / S0
     * where S1 and S2 are the second and third component of the Stokes vector.
     * The AoP can be computed as:
     *                      AoP = 0.5 * atan2 (S2 / S1)
     *
     * @arg stokesVector: Vector with 3 components. Each of them corresponds to
     * the images of S0, S1 and S2, respectively.
     *
     * @arg output: Vector with three components: Intensity - AoP - DoLP.
    */
    static void computeIRoPhi(std::vector<cv::Mat>& stokesVector, std::vector<cv::Mat> &output);

    /**
     * @brief convertFloatToIntsImgs: Convert a set of images to integer type.
     *  It will execute a for loop, in which the image intensitites are scaled
     * by a given factor, and converted to the provided OpenCV type. The converted
     * images are placed in the same images input vector.
     *
     * @arg imgs: Vector with the images that we want to convert to integer types.
     * @arg scaling: Scale factors for each image. The order of these scaling factors
     *  must be the same as the vector of images.
     * @arg intType: Final image type (CV_8UC1, CV_16UC1).
    */
    static void convertFloatToIntsImgs(std::vector<cv::Mat>& imgs, std::vector<double> scaling, int intType);

    /**
     * @brief getColorMapJetinRGB: Get the color palette that passes from grayscale
     * to JET. This conversion is done by creating a linear vector of type cv::Mat,
     * and then applying it the OpenCV Jet palette. Then this palette is converted
     * to RGB type.
    */
    static cv::Mat getColorMapJetinRGB();

    /**
     * @brief getColorMapHSVinRGB: Get color palette that passes from grayscale
     * to HSV. This conversion is done by creating a linear vector of type cv::Mat,
     * and then filling it with values of 255 for saturation and value, and an
     * increasing value of HSV, from 0 until 180.
    */
    static cv::Mat getColorMapHSVinRGB();

};
#endif // __POLARIMETRIC_IMAGES_PROCESSING_HPP__