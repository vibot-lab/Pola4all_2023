#ifndef __CAMERA_TYPES_HPP__
#define __CAMERA_TYPES_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes
#include <map>

// Qt Includes

// Pylon includes

// Custom includes

/**
 * @brief CameraState struct
 *
 *  Struct that holds all the parameters that define a camera state:
 *      - pixel gain
 *      - exposure time
 *      - frame rate
 *      - color gains
 *      - auto gain state
 *      - auto exposure state
 *      - auto white balance state
 *  This struct serves for ease the transport of the camera state
*/
typedef struct {
    double gain;
    double exposureUs;
    int frameRate;
    double redGain;
    double greenGain;
    double blueGain;
    bool autoGainEnabled;
    bool autoExposureEnabled;
    bool autoWhiteBalanceEnabled;

} CameraState;

/**
 * @brief bitDepthStruct: Base class that hold all the format information
 *   required to know the incoming image format. This class can be used also
 *   to set the camera in the right output image format. Using this struct
 *   allows to avoid being compatible with a single image format, and be flexible
 *   when changing from one format to another. Furthermore, adding a new pixel
 *   format can be done trivially: just declare a class that inherits from this class,
 *   and put on it the required values.
 *
 * @arg opencvType: OpenCV bit-depth value (CV_8U or CV_16U).
 * @arg bithDepth: Integer value. Image bit-depth (8 or 16).
 * @arg maxAllowedVal: Maximum allowed value. For 8-bits it is 255, and for 16-bits
 *   it is 4095, due to the sensors maximum bit-depth (so far) are of 12 bits.
*/
class bitDepthStruct
{
public:
    bitDepthStruct(int opencvType,
        int bitDepth,
        float maxAllowedVal) :
    _opencvType(opencvType),
    _bitDepth(bitDepth),
    _maxAllowedVal(maxAllowedVal){}

    const int _opencvType;
    const int _bitDepth;
    const float _maxAllowedVal;
};

/**
 * @brief bitDepthStruct_16u: Implementation of bitDepthStruct for 12-bits camera.
*/
class bitDepthStruct_12u : public bitDepthStruct
{
public:
    bitDepthStruct_12u() : bitDepthStruct(CV_16U, 12, 4095.0) {}
};

/**
 * @brief bitDepthStruct_8u: Implementation of bitDepthStruct for 8-bits camera.
*/
class bitDepthStruct_8u : public bitDepthStruct
{
public:
    bitDepthStruct_8u() : bitDepthStruct(CV_8U, 8, 255.0){}
};

/**
 * @brief ProcessingLevel enum
 *  This enum contains the enumeration of the different type of processing we can do
 * at the images received by the polarimetric camera. These level means:
 *      - RAW_IMAGE: Do not process the image, leave it as received from the
 *          camera driver (all color and polarization information put in a single image)
 *      - RAW_SPLITTED_IMAGES: Divide the received image into 4,
 *          one per polarization state. No further processing is done, so the
 *          four images are single channel.
 *      - COLOR_SPLITTED: After dividing the images into 4 images, single channel,
 *          we proceed to demosaick them and to create color images (3 channels).
 *      - COLOR_CORRECTED: Same as before, but the intensities are computed based
 *          on the calibration matrix provided. So, from the input intensities and
 *          the calibration matrix, we compute the Stokes vector, and then, from
 *          the Stokes vector, we compute the ideal super-pixel intensities.
 *      - COLOR_ORIGINAL: It means split the images, and create the intensity
 *          color image, as if it were taken by normal camera. This is done using
 *          the I0 and I90 images.
 *      - SIM_POLARIZER: Simulate the effect of placing a linear polarizer in front
 *          of a normal camera. This way, we can analyze if there is an angle in which
 *          the bright points disappear. The angle can be provided as one of the
 *          processing level parameters.
 *      - FAKE_COLORS: Take the S0, DoLP and AoLP and place them into a HSV image. Then
 *          we convert it into BGR image to show it.
 *      - REMOVE_SPECULARITY: It will take the input image, compute the Stokes
 *          parameters, and with them, compute the AoLP. Then, we apply a linear
 *          polarisation filtering, with an orientation the AoLP + 90 degrees.
 *          It repeats this procedure pixelwise, and with that, we can erase the
 *          specularity in the image.
 *      - STOKES: It receives a raw image, and it takes the pixels corresponding to
 *          a single color super-pixel to compute the stokes S0, S1 and S2 parameters
 *          for that wave length. For that, it uses either a matrix with the coefficients
 *          that multiplies each intensity value in order to obtain the corresponding
 *          Stokes values per pixel. Each channel is returned separatedly. It outputs
 *          9 images.
 *      - I_RO_PHI: It does the same as STOKES, but it uses the output Stokes images
 *          to compute the intensity, degree of polarization and angle of polarization
 *          images. The returned images are splitted by channel, and the AoP and DoP
 *          images are colored with palettes. It outputs 9 images.
 *      - I_RO_PHI_RAW: It does the same as STOKES, but it uses the output Stokes images
 *          to compute the intensity, degree of polarization and angle of polarization
 *          images. The returned images are not demosaicked, but split by color.
 *          It outputs 9 images.
 *
*/
typedef enum {
    RAW_IMAGE = 0,
    RAW_SPLITTED_IMAGES,
    COLOR_SPLITTED,
    COLOR_ORIGINAL,
    SIM_POLARIZER,
    FAKE_COLORS,
    REMOVE_SPECULARITY,
    STOKES,
    I_RO_PHI,
    RAW_I_RO_PHI,
    /// WARNING: Do not put more enums after this case
    NUM_PROCESSING_OPT,
} ProcessingLevel;

/**
 * @brief Struct that holds all the parameters required to process a polarization
 * image. This way, we do not have to modify the interfaces each time we add
 * a new parameters to the arguments list. Additionally, this helps to future
 * modifications, since adding a parameters does not change anything from the
 * rest of the code.
 *
 * @param action: ProcessingLevel enum value that tells which operations the
 *      user wants to do after retriving an image from the camera.
 *
 * @param aop: Angle of polarization in degrees. It is used only for the
 *      SIM_POLARIZER mode. It represents the orientation of a linear polarisation filter
 *      placed in front of a normal RGB camera, in order to filter out the specular light.
 * @param filtersMap: Map from the filter orientations to their position in the super-pixel
 *      configuration. For instance, for the Basler acA2440-75ucPOL, the map is:
 *      135 --> 0
 *      0   --> 1
 *      90  --> 2
 *      45  --> 3
*/
typedef struct {
    ProcessingLevel action;
    int aop;
    std::map<int, int> filtersMap;
    std::shared_ptr<bitDepthStruct> imgFormatData;
} ProcessingParameters;

/**
 * @brief CameraParams enum
 *
 *  This enum contains the different types of parameters we can request / change
 * in the camera. With it, it is easy to decide which parameter to read / write
 * from the struct. This simplifies the function prototypes: We provide the
 * complete struct, and the parameter we want from it. This way, all the setters
 * and getters that involve the camera parameters have the same arguments.
 *
 *  For the ROS implementation, this is also convinient, since we do not need to
 * handle several types of messages, the same message is used, and an additional
 * field is send in order to know to which parameter we refer to.
 *      - ALL: All the parameters in the CameraState struct
 *      - AUTO_EXPOSURE --> autoExposureEnabled
 *      - AUTO_GAIN --> autoGainEnabled
 *      - AUTO_WHITE_BALANCE --> autoWhiteBalanceEnabled
 *      - FRAME_RATE --> frameRate
 *      - EXPOSURE_TIME --> exposureUs
 *      - GAIN --> gain
 *      - WHITE_BALANCE_GAINS --> redGain, greenGain, blueGain
*/
typedef enum {
    ALL=0,
    AUTO_EXPOSURE,
    AUTO_GAIN,
    AUTO_WHITE_BALANCE,
    FRAME_RATE,
    EXPOSURE_TIME,
    GAIN,
    WHITE_BALANCE_GAINS,
    // DO NOT ADD MACROS AFTER NUM_OF_PARAMS
    NUM_OF_PARAMS
} CameraParams;

typedef struct {
    cv::Mat averageImage;
    int anglePol;
    int N;
} experimentSamples;

/**
 * @brief Calibrators enum
 * This enum provides the different flat field calibrators ids. This way, we can
 * use them to do a switch - case statement to know which calibrator module
 * the user requires in order to do the flat-field calibration.
 * The implemented modules are:
 * - PIXEL_GAIN: It involves only the computation of the pixel gains correction
 * - BLACK_CURRENT: It involves only the computation offset required to make
 * the pixel values to reach the zero value.
*/
typedef enum {
    PIXEL_GAIN = 0,
    BLACK_CURRENT,
    /// WARNING: Do not put more enums after this case
    NUM_CALIBRATORS,
} Calibrators;

/**
 * @brief CosineDescriptor: Struct with all the coefficients of the cosine function
 * that we use to fit when we do the pixel gain calibration. The equation used is
 *                  I(v) = (S0 * Ti / Pi) * (1 + dop * Pi * cos (2*(v - phaseShift)))
 * where:
 *  - v is the input angle at which we want to compute the cosine function
 *  - S0 is the intensity of the unpolarized light
 *  - Ti is the transfer factor (ideally, 0.5)
 *  - Pi is the factor that considers the non ideality of the pixel
 *  - DoP: Degree of polarization that is tranfered through the pixel.
 *  - phaseShift is the orientation of the micro polarizer in the pixel.
 *   we cannot split S0, Ti and Pi, nor dop and Pi, we combine them as:
 *                      amplitude = S0 * Ti / Pi
 *                      nonIdeality = dop * Pi
*/
typedef struct
{
    double amplitude;
    double phaseShift;
    double nonIdeality;
} CosineDescriptor;

/**
 * @brief SuperPixelParams: Struct with all the coefficients of the cosine function
 * that we use to fit when we do the super-pixel gain calibration. The equation used is
 *                  Ii(v) = (S0 * Ti / Pi) * (1 + dop * Pi * cos (2*(v - phaseShift_i)))
 * where:
 *  - i indicates the angle of the considered pixel (i = {135, 0, 90, 45})
 *  - v is the input angle at which we want to compute the cosine function
 *  - S0 is the intensity of the unpolarized light
 *  - Ti is the transfer factor (ideally, 0.5)
 *  - Pi is the factor that considers the non ideality of the pixel
 *  - DoP: Degree of polarization that is tranfered through the pixel.
 *  - phaseShift_i is the orientation of the micro polarizer in the pixel.
 *   we cannot split S0, Ti and Pi, nor dop and Pi, we combine them as:
 *                      TiPiS0_i = S0 * Ti / Pi
 *                      TiS0DOP_i = dop * Pi
 *
 *  Since the superpixel consists of 4 pixels, phaseShift, TiPiS0 and TiS0DOP
 * are vectors with 4 elements.
*/
typedef struct
{
    std::vector<double> phaseShift;
    std::vector<double> TiPiS0;
    std::vector<double> TiS0DOP;
} SuperPixelParams;

/**
 * @brief LightEstimationMethod: Enum that defines how we want to estimate the
 * light parameters from the light source. Since we cannot separate the values
 * Ti and Pi from S0 and the degree of polarization if we do not know the last
 * two, we have to find a solution to know them from the samples taken from
 * the camera.
 *  In all the method, we consider the camera as ideal (Ti = 0.5 and Pi = 1.0),
 * and we estimate S0 and the DoLP by different ways:
 *      - We take do the running average of S0 and DoLP detected by the pixels.
 *      - We take the detected values from the pixel that detects the maximum S0
 *      - We take the detected values from the pixel that detects the median value of S0.
 *
 *  All the methods should provide the same distribution of values of Ti and Pi,
 * but we will shift their position. The median is better than the average
 * since it is more robust to bad measurements, but it is possible to have degrees
 * of polarization detected by the camera higher than 1. The maximum value is
 * sensible to noise, but it avoids having DoLP values higher than 1.
*/
typedef enum
{
    METHOD_AVERAGE = 120,
    METHOD_MAXIMUM = 125,
    METHOD_MEDIAN = 130,
} LightEstimationMethod;

/**
 * @brief DistributionPlotType: List of distribution plots availables.
 *   This enum is used to decide which data we have to retrieve from the calibrator
 *  in the MainWindow. The switch between each type of plot is done in the
 *  MatPlotLibPlotsWidget module.
*/
typedef enum
{
    GAIN_PLOT = 50,
    PHASE_PLOT = 55,
    TI_PLOT = 65,
    PI_PLOT = 70,
    BLACK_CURRENT_PLOT = 75,
} DistributionPlotType;

/**
 * @brief LiveDistributionPlotType: This enum defines the different live plot types.
 * A live plot is a distribution plot of the polarimetric parameters of the current image
 * begin shown in the software. For instance, we can plot the image intensity
 * distribution, or the angle of polarization distribution.
*/
typedef enum
{
    INTENSITY_PLOT = 200,
    DOP_PLOT = 205,
    AOP_PLOT = 210,
} LiveDistributionPlotType;

/**
 * @brief ColorChannelType: This datatype is used for the Row Live Plot, that
 * makes use of the MatPlotLib module. These labels are used to decide which
 * color channel will be plot. The assigned numbers corresponds to the index
 * of the output vector in the PolarimetricImageProcessing class when the mode
 * is I - Rho - Phi or Raw I - Rho - Phi.
 */
typedef enum
{
    RED_CHANNEL = 0,
    GREEN_CHANNEL_1 = 1,
    GREEN_CHANNEL_2 = 2,
    BLUE_CHANNEL = 3,
} ColorChannelType;

#endif // __CAMERA_TYPES_HPP__
