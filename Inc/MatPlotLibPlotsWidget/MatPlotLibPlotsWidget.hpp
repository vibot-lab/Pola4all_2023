#ifndef __MATPLOTLIB_PLOTS_WIDGET_HPP__
#define __MATPLOTLIB_PLOTS_WIDGET_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes
#include <string>
#include <vector>

// Qt Includes
#include <QCheckBox>
#include <QGroupBox>
#include <QSpinBox>
#include <QWidget>

// Custom includes
#include "CameraTypes.hpp"
#include "MatPlotLibPlotsWidget/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

/**
 * @brief MatPlotLibPlotsWidget: Widget that shows different type of plots of
 * the calibration data. The options availables are:
 *  - Plot a set of samples: Since a single image is composed by 5 x 10^6 pixels,
 *  we cannot show them all at once. Thus, we show the values contained in a
 * single super-pixel, which is formed by 16 pixels (color and polarization),
 * arranged in a matrix of 4x4. We can choose an offset value in rows and columns
 * in order to decide which group of 4x4 pixels we are interested in.
 *
 * - Plot a distribution of values: From the curve fitting step,
 * we obtain several parameters per pixel: Gain, phase shift, Ti and Pi.
 * Those parameters will have different values, according to the pixel we consider.
 * This option shows how those values are distributed. Using 100 bins, we take
 * the minimum and maximum values of that parameter, and we split this interval in
 * 100 pieces. This function is also used to draw the current image polarization parameters
 * distribution (AoLP, DoLP and Intensity).
 *
 *   Since MatPlotLib is not optimized to show a lot of plots at the time, each time
 * a new plot is triggered, the previous plots will be closed. An specific button
 * is added to close all the plots at once too.
*/
class MatPlotLibPlotsWidget : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     *
     * @arg plotFontSize: Font size used for the distribution plots.
     * @arg plotLabelSize: Label size used for the distribution plots.
     * @arg imgFormat: Struct with the image bit-depth information (max allowed
     *  value, OpenCV datatype, and bit-depth as integer).
    */
    MatPlotLibPlotsWidget(int plotFontSize, int plotLabelSize, std::shared_ptr<bitDepthStruct> imgFormat, QWidget* parent = 0);
    ~MatPlotLibPlotsWidget();

    /**
     * @brief createValueDistributionPlot: Show a set of plots, where each plot constains
     * the distribution of the values.
     *
     * @arg pixelParameterImage: Array with the images from which we want to
     *  create the distribution plots.
     *
     * @arg pixelIdsStrings: Array with strings that identify the pixel. This
     *  vector contains as many elements as the amount of images in the
     *  pixelParameterImage vector. The order of the labels should be the same as
     *  for pixelParameterImage.
     *
     * @arg paramType: String that identifies the type of parameter we are plotting.
     *
     * @arg distMaxVal: Maximum value in the X-axis that will be included in
     *   the distribution plot.
    */
    void createValueDistributionPlot(
        const std::vector<cv::Mat>& pixelParameterImage,
        const std::vector<std::string>& pixelIdsStrings,
        std::string paramType,
        double distMaxVal);

    /// \brief getLivePlotMaxVal: Get the X-axis maximum value allowed in the live distribution
    double getLivePlotMaxVal() {return _liveDistMaxVal;}

    /// \brief getLivePlotMaxVal: Get the X-axis maximum value allowed in the selected parameter distribution
    double getParamDistMaxVal() {return _distributionPlotMaxVal;}

    /**
     * @brief createSamplesPlot: Plot the samples of a set of pixels. Since the
     * super-pixel is composed by several pixels, this function will show
     * as many pixels as the size of the super-pixel. Using the corresponding
     * QSpinBoxes, we can change the set of pixels to plot.
     * The values to be plot are obtained after taking several samples of a linearly
     * polarized light, at different angles.
     *
     * @arg angles: The angles at which we have taken the samples.
     *
     * @arg avgImgs: Image with several channels. The ith channel is the
     *  intensity sample at the ith angle.
    */
    void createSamplesPlot(const std::vector<double>& angles, const cv::Mat& avgImgs);

    /**
     * @brief setEnabled: It will enable or disable certain widgets in
     * the module.
     *
     * @arg enabled: Boolean. If true, the widgets are enabled.
    */
    void setEnabled(bool enabled);

    /**
     * @brief closePlots: Close all the plots. If there is no plot shown,
     *   this function will do nothing.
    */
    void closePlots();

    /**
     * @brief updateImageSize: The limits of the samples we can show in the "Samples Plot"
     *   function depends on the image size. This function must be called each time
     *   the dataset of sample images to be shown is changed.
     *
     * @arg rows: Amount of rows in the images.
     * @arg cols: Amount of columns in the images.
    */
    void updateImageSize(int rows, int cols);

    /**
     * @brief plotLiveRow: Generate the plots of intensity, AoLP and DoLP of
     *   a single line of image. The line plot corresponds to the color channel
     *   specified in ch (RED_CHANNEL, GREEN_1_CHANNEL, GREEN_2_CHANNEL, BLUE_CHANNEL),
     *   and the line is specified in the SpinBox that is in this widget.
     *
     * @param polaImgs Vector with four color channels of the polarization images
     *   (intensity, AoLP, DoLP). They must be single channel images.
     * @param ch: Color channel to be plot.
     */
    void plotLiveRow(std::vector<cv::Mat> polaImgs, ColorChannelType ch);

signals:
    /**
     * @brief requestDistributionData: Signal emitted each time the user clicks the
     *  "Plot calibration values distribution" button.
    */
    void requestDistributionData(DistributionPlotType type);

    /**
     * @brief requestSamples: Signal emitted each time the user clicks the
     *  "Plot samples" button.
    */
    void requestSamples();

    /**
     * @brief requestLiveDistributionPlot: Signal each time the user clicks the
     * button "Plot polarimetric distribution". It tells which current image
     * distribution plot the user wants.
     *
     * @arg type: Flag that indicates the desired type of plot.
    */
    void requestLiveDistributionPlot(LiveDistributionPlotType type);

    /**
     * @brief switchRawPlot: Signal emitted each time the QCheckBox of the live
     *  plot changes its state.
     * @param enabled Boolean. If true, the live plot has been enabled by the user.
     */
    void switchRowPlot(bool enabled);

protected:
    /**
     * @brief onRequestDistributionData: Slot called each time the user wants
     *   to plot a distribution of calibration parameters.
    */
    void onRequestDistributionData();

    /// \brief onGainPlotSelected: Slot called each time the Gain plot radio button is toggled
    void onGainPlotSelected(bool state);

    /// \brief onPhasePlotSelected: Slot called each time the Phase plot radio button is toggled
    void onPhasePlotSelected(bool state);

    /// \brief onTiPlotSelected: Slot called each time the Ti plot radio button is toggled
    void onTiPlotSelected(bool state);

    /// \brief onPiPlotSelected: Slot called each time the Pi plot radio button is toggled
    void onPiPlotSelected(bool state);

    /**
     * @brief onBlackCurrentPlotSelected: Slot called each time the black current
     *   plot radio button is toggled.
    */
    void onBlackCurrentPlotSelected(bool state);

    /// \brief onAoPLivePlotRequested: Slot called each time the AoLP live plot radio button is toggled
    void onAoPLivePlotRequested(bool state);

    /// \brief onDoPLivePlotRequested: Slot called each time the DoLP live plot radio button is toggled
    void onDoPLivePlotRequested(bool state);

    /**
     * @brief onIntensityLivePlotRequested: Slot called each time the intensity
     *  live plot radio button is toggled
    */
    void onIntensityLivePlotRequested(bool state);

    /// \brief onRequestLiveDistributionPlot: Slot called each time the user wants to plot a the current image distribution.
    void onRequestLiveDistributionPlot();

    /**
     * @brief onSwitchRunningState: Slot called each time the user changes the
     *  Row Live plot QCheckBox state. When called, the signal switchRowPlot is
     *  emitted, all the plots are closed, all the other plots options are
     *  enabled when the live plot is enabled, vice-versa.
     *  Additionally, when this plot is enabled, three figures are created and shown.
     *
     * @param state: New state of the QCheckBox.
     */
    void onSwitchRunningState(int state);

private:
    /**
     * @brief initializeWidget: Initialize the widget structure.
    */
    void initializeWidget();

    /**
     * @brief addLivePlots: Part of the widget initialization. It will add
     *  the widgets required to implement the live plots functionality.
     *
     * @param mainLAyout Pointer to the layout to which the live widget will be added.
     */
    void addLivePlots(QLayout* mainLAyout);

    /**
     * @brief addDistributionPlots: Part of the widget initialization. It will add
     *  the widgets required to implement the calibrated parameters distributions.
     *
     * @param mainLAyout Pointer to the layout to which the parameters distributions
     *  widget will be added.
     */
    void addDistributionPlots(QLayout* mainLAyout);

    /**
     * @brief addSamplePlots: Part of the widget initialization. It will add
     *  the widgets required to implement the samples visualization.
     *
     * @param mainLAyout Pointer to the layout to which the samples visualization
     *  widget will be added.
     */
    void addSamplePlots(QLayout* mainLAyout);

    /**
     * @brief addLiveRowMode: Add the QGroupBox that corresponds to the live row
     *   plot feature. This feature consists of a QSpinBox to choose the line
     *   we want to plot and a QCheckBox to enable or disable it. When this feature
     *   is enabled, all the other plot features are disabled.
     *
     * @param mainLayout: Main layout to which we will add this new feature widget.
     */
    void addLiveRowMode(QLayout* mainLayout);

    /**
     * @brief setEnableRunningState: Enable or disable all the plots feautures,
     *   except for the live row plot feature.
     *
     * @param enabled: If true, the plot features are enabled.
     */
    void setEnableRunningState(bool enabled);

    /**
     * @brief computeHistogram: Compute the value distribution of an image. This
     *  function is thought to work with float values, but it can also work
     *  with integer images. It converts the image into a vector, it sorts them,
     *  and having the bin limits vector, we check when the vector data passes
     *  the different threshold. Even though this implementation looks more complex
     *  than counting, it is optimal: There is no loop that goes pixel by pixel,
     *  and we do not do a series of if statements to check in which bin the values
     *  have to be. Only one for loop is required, and it only covers 100 iterations.
     *
     * @arg data: Input image from which we want to compute the histogram.
     * @arg amountBins: Amount of bins in which we want to split the histogram interval.
     * @arg minVal: Minimum value of the histogram
     * @arg maxVal: Maximum value of the histogram
     * @arg outputTicks [output]: X axis values of the histogram.
     * @arg outputHist [output]: Y axis values of the histogram.
    */
    void computeHistogram(
        const cv::Mat& data,
        const int& amountBins,
        const double& minVal,
        const double& maxVal,
        std::vector<double>& outputTicks,
        std::vector<double>& outputHist);

    /**
     * @brief plotHistogram: Create a bars plot that represents the histogram
     *  of the given image.
     *
     * @arg data: Image from which we want to compute the histogram.
     * @arg bins: Amount of bins in which we want to split the histogram interval.
     * @arg minX: Minimum value of the histogram
     * @arg maxX: Maximum value of the histogram
     * @arg title: Title we want to put in the plot
    */
    void plotHistogram(cv::Mat data, int bins, double minX, double maxX, std::string title);

    /**
     * @brief convertMatToDoubleVector: Transform a single channel image into
     *  a vector of doubles. This implementation is independent of the datatype
     *  of the matrix. The handled OpenCV datatypes are: CV_8U, CV_16U, CV_32F, CV_64F
     *
     * @arg img: Single channeled image
     *
     * @returns: Vector of doubles with the image pixel values on it.
     */
    std::vector<double> convertMatToDoubleVector(const cv::Mat& img);

    /**
     * @brief openFigures: Close all opened figures, and open the figures for the real-time
     *  row plot.
    */
    void openFigures();

    QGroupBox* _livePlotGroup;
    QGroupBox* _rowLivePlotGroup;
    QGroupBox* _samplesGroup;
    QGroupBox* _distrGroup;
    QSpinBox* _pixelOffsetRows;
    QSpinBox* _pixelOffsetCols;
    QSpinBox* _rowToPlot;
    QCheckBox* _isLiveRowRunning;
    DistributionPlotType _chosenDistPlot;
    LiveDistributionPlotType _chosenLivePlot;
    double _distributionPlotMaxVal;
    double _liveDistMaxVal;

    int _histAmountBins;

    int _plotFontSize;
    int _plotLabelSize;

    int _intensityFigNum;
    int _aolpFigNum;
    int _dolpFigNum;

    std::shared_ptr<bitDepthStruct> _imgFormat;
};

#endif // __MATPLOTLIB_PLOTS_WIDGET_HPP__