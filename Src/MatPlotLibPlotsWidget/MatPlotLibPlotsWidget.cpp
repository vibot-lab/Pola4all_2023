// OpenCV includes

// STD includes
#include <algorithm>

// Qt Includes
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>

// Custom includes
#include "MatPlotLibPlotsWidget/matplotlib-cpp/matplotlibcpp.h"
#include "MatPlotLibPlotsWidget/MatPlotLibPlotsWidget.hpp"

namespace plt = matplotlibcpp;

constexpr int DISTRIBUTION_GAIN_MAX_VAL = 2;
constexpr int DISTRIBUTION_PHASE_MAX_VAL = 180;
constexpr double DISTRIBUTION_NON_IDEALITY_MAX_VAL = 1.2;

template<typename T>
std::vector<T> convertToVector(cv::Mat mat)
{
    std::vector<T> array;
    if (mat.isContinuous())
    {
        array.assign((T*)mat.data, (T*)mat.data + mat.total() * mat.channels());
    }
    else
    {
        for (int i = 0; i < mat.rows; ++i)
        {
            array.insert(array.end(), mat.ptr<T>(i), mat.ptr<T>(i) + mat.cols * mat.channels());
        }
    }
    return array;
}

MatPlotLibPlotsWidget::MatPlotLibPlotsWidget(int plotFontSize, int plotLabelSize, std::shared_ptr<bitDepthStruct> imgFormat, QWidget* parent) :
    QWidget(parent),
    _livePlotGroup(nullptr),
    _rowLivePlotGroup(nullptr),
    _samplesGroup(nullptr),
    _distrGroup(nullptr),
    _pixelOffsetRows(nullptr),
    _pixelOffsetCols(nullptr),
    _rowToPlot(nullptr),
    _isLiveRowRunning(nullptr),
    _histAmountBins(100),
    _plotFontSize(plotFontSize),
    _plotLabelSize(plotLabelSize),
    _intensityFigNum(-1),
    _aolpFigNum(-1),
    _dolpFigNum(-1),
    _imgFormat(imgFormat)
{
    plt::backend("Qt5Agg");
    plt::ion();
    initializeWidget();
}

MatPlotLibPlotsWidget::~MatPlotLibPlotsWidget()
{
    plt::close();
    delete _pixelOffsetRows;
    delete _pixelOffsetCols;
    delete _rowToPlot;
    delete _isLiveRowRunning;
    delete _livePlotGroup;
    delete _rowLivePlotGroup;
    delete _samplesGroup;
    delete _distrGroup;
}

void MatPlotLibPlotsWidget::initializeWidget()
{
    QVBoxLayout* layout = new QVBoxLayout();

    addSamplePlots(layout);
    addDistributionPlots(layout);
    addLivePlots(layout);
    addLiveRowMode(layout);

    QPushButton* closePlotsButton = new QPushButton("Close all plots", this);
    layout->addWidget(closePlotsButton);
    layout->addStretch(1);

    connect(closePlotsButton,
        &QPushButton::clicked,
        this,
        &MatPlotLibPlotsWidget::closePlots);

    setLayout(layout);
}

void MatPlotLibPlotsWidget::addLiveRowMode(QLayout* mainLAyout)
{
    _rowLivePlotGroup = new QGroupBox("Row live plot", this);
    QVBoxLayout* groupLayout = new QVBoxLayout(_rowLivePlotGroup);

    _isLiveRowRunning = new QCheckBox("Start live row plot", _rowLivePlotGroup);

    QWidget* rowWidget = new QWidget(_rowLivePlotGroup);
    QHBoxLayout* formLayout = new QHBoxLayout(rowWidget);
    QLabel* rowText = new QLabel("Row position % (0 - 100): ", rowWidget);
    _rowToPlot = new QSpinBox(rowWidget);
    _rowToPlot->setMinimum(0);
    _rowToPlot->setMaximum(99);
    _rowToPlot->setValue(50);
    formLayout->addWidget(rowText);
    formLayout->addWidget(_rowToPlot);
    rowWidget->setLayout(formLayout);

    groupLayout->addWidget(_isLiveRowRunning);
    groupLayout->addWidget(rowWidget);
    _rowLivePlotGroup->setLayout(groupLayout);

    mainLAyout->addWidget(_rowLivePlotGroup);
    connect(_isLiveRowRunning,
        &QCheckBox::stateChanged,
        this,
        &MatPlotLibPlotsWidget::onSwitchRunningState);
}

void MatPlotLibPlotsWidget::addLivePlots(QLayout* mainLAyout)
{
    // -------- Live plot group --------
    _livePlotGroup = new QGroupBox("Live plots", this);
    QVBoxLayout* liveDistLayout = new QVBoxLayout(_livePlotGroup);

    QPushButton* plotLiveDistButton = new QPushButton("Plot polarimetric distribution", _livePlotGroup);
    QRadioButton* radioS0 = new QRadioButton("Intensity", _livePlotGroup);
    QRadioButton* radioDOP = new QRadioButton("DoLP", _livePlotGroup);
    QRadioButton* radioAOP = new QRadioButton("AoLP", _livePlotGroup);

    _chosenLivePlot = INTENSITY_PLOT;
    radioS0->setChecked(true);
    _liveDistMaxVal = _imgFormat->_maxAllowedVal;

    liveDistLayout->addWidget(radioS0);
    liveDistLayout->addWidget(radioDOP);
    liveDistLayout->addWidget(radioAOP);
    liveDistLayout->addWidget(plotLiveDistButton);
    _livePlotGroup->setLayout(liveDistLayout);

    mainLAyout->addWidget(_livePlotGroup);

    connect(radioS0,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onIntensityLivePlotRequested);

    connect(radioDOP,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onDoPLivePlotRequested);

    connect(radioAOP,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onAoPLivePlotRequested);

    connect(plotLiveDistButton,
        &QPushButton::clicked,
        this,
        &MatPlotLibPlotsWidget::onRequestLiveDistributionPlot);
}

void MatPlotLibPlotsWidget::addDistributionPlots(QLayout* mainLayout)
{
    /// -------- Distribution plots selection --------
    _distrGroup = new QGroupBox("Distribution plot", this);
    QVBoxLayout* distribLayout = new QVBoxLayout(_distrGroup);

    QPushButton* plotDistButton = new QPushButton("Plot calibration values distribution", _distrGroup);
    QRadioButton* radio1 = new QRadioButton("Gain", _distrGroup);
    QRadioButton* radio2 = new QRadioButton("Phase", _distrGroup);
    QRadioButton* radio3 = new QRadioButton("Ti", _distrGroup);
    QRadioButton* radio4 = new QRadioButton("Pi", _distrGroup);
    QRadioButton* radio5 = new QRadioButton("Black current", _distrGroup);

    _chosenDistPlot = GAIN_PLOT;
    radio1->setChecked(true);
    _distributionPlotMaxVal = DISTRIBUTION_GAIN_MAX_VAL;

    distribLayout->addWidget(radio1);
    distribLayout->addWidget(radio2);
    distribLayout->addWidget(radio3);
    distribLayout->addWidget(radio4);
    distribLayout->addWidget(radio5);
    distribLayout->addWidget(plotDistButton);
    _distrGroup->setLayout(distribLayout);

    mainLayout->addWidget(_distrGroup);

    connect(radio1,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onGainPlotSelected);

    connect(radio2,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onPhasePlotSelected);

    connect(radio3,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onTiPlotSelected);

    connect(radio4,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onPiPlotSelected);

    connect(radio5,
        &QRadioButton::toggled,
        this,
        &MatPlotLibPlotsWidget::onBlackCurrentPlotSelected);

    connect(plotDistButton,
        &QPushButton::clicked,
        this,
        &MatPlotLibPlotsWidget::onRequestDistributionData);
}

void MatPlotLibPlotsWidget::addSamplePlots(QLayout* mainLayout)
{
    // -------- Plot samples button --------
    _samplesGroup = new QGroupBox("Sample plots", this);
    QVBoxLayout* samplesLayout = new QVBoxLayout(_samplesGroup);
    QPushButton* plotSamplesButton = new QPushButton("Plot samples", _samplesGroup);

    // We create a single widget with the two spinboxes in a QFormLayout (label, widget).
    QWidget* offsetWidget = new QWidget(_samplesGroup);

    // We create the spin box that holds the offset in rows
    _pixelOffsetRows = new QSpinBox(offsetWidget);
    _pixelOffsetCols = new QSpinBox(offsetWidget);

    // We set zero at initialization, but these values are changed on
    // the slot updateImageSize, based on the image size.
    _pixelOffsetRows->setMinimum(0);
    _pixelOffsetCols->setMinimum(0);
    _pixelOffsetRows->setMaximum(0);
    _pixelOffsetCols->setMaximum(0);

    QLabel* offsetRowsLabel = new QLabel("Offset in Rows:", offsetWidget);
    QLabel* offsetColsLabel = new QLabel("Offset in Columns:", offsetWidget);
    QFormLayout* offsetFormLayout = new QFormLayout(offsetWidget);
    offsetFormLayout->addRow(offsetRowsLabel, _pixelOffsetRows);
    offsetFormLayout->addRow(offsetColsLabel, _pixelOffsetCols);
    offsetWidget->setLayout(offsetFormLayout);

    samplesLayout->addWidget(offsetWidget);
    samplesLayout->addWidget(plotSamplesButton);
    _samplesGroup->setLayout(samplesLayout);

    mainLayout->addWidget(_samplesGroup);

    connect(plotSamplesButton,
        &QPushButton::clicked,
        this,
        &MatPlotLibPlotsWidget::requestSamples);
}

void MatPlotLibPlotsWidget::updateImageSize(int rows, int cols)
{
    if (rows > 5 && cols > 5)
    {
        _pixelOffsetRows->setMaximum(rows - 5);
        _pixelOffsetCols->setMaximum(cols - 5);
    }
    else
    {
        _pixelOffsetRows->setMaximum(0);
        _pixelOffsetCols->setMaximum(0);
    }
}

void MatPlotLibPlotsWidget::setEnabled(bool enabled)
{
    if (!enabled)
    {
        _isLiveRowRunning->setChecked(false);
    }
    _livePlotGroup->setEnabled(enabled);
    _rowLivePlotGroup->setEnabled(enabled);
}

std::vector<double> MatPlotLibPlotsWidget::convertMatToDoubleVector(const cv::Mat& img)
{
    uchar depth = img.type() & CV_MAT_DEPTH_MASK;
    assert(img.channels() == 1);
    std::vector<double> output;
    switch(depth)
    {
        case CV_8U:
        {
            output = std::vector<double>(img.begin<uchar>(), img.end<uchar>());
            break;
        }
        case CV_16U:
        {
            output = std::vector<double>(img.begin<ushort>(), img.end<ushort>());
            break;
        }
        case CV_32F:
        {
            output = std::vector<double>(img.begin<float>(), img.end<float>());
            break;
        }
        case CV_64F:
        {
            output = std::vector<double>(img.begin<double>(), img.end<double>());
            break;
        }
        default:
        {
            std::cout << "ERROR: Data type not handled when converting image into vector" << std::endl;
            assert(0);
            break;
        }
    }
    return output;
}

void MatPlotLibPlotsWidget::computeHistogram(
    const cv::Mat& data,
    const int& amountBins,
    const double& minVal,
    const double& maxVal,
    std::vector<double>& outputTicks,
    std::vector<double>& outputHist)
{
    double delta = (maxVal - minVal) / amountBins;
    delta = std::max(delta, 0.001);

    // In order to convert the cv::Mat into a pointer, we need to ensure it is
    // a continuous block of memory. Clone will ensure this.
    cv::Mat image;
    if (!data.isContinuous())
    {
        image = data.clone();
    }
    else
    {
        image = data;
    }

    std::vector<double> imageVector = convertMatToDoubleVector(image);

    // Now we have the image as a sorted vector, this will ease the histogram computation.
    std::sort(imageVector.begin(), imageVector.end());

    // We create a vector with the value limits
    outputTicks = std::vector<double>(amountBins, 0);
    outputHist = std::vector<double>(amountBins, 0);

    int64_t counter = 0;
    for (int i = 0; i < amountBins; i++)
    {
        outputTicks[i] = ((i+1) * delta) + minVal;
        outputHist[i] = std::lower_bound(imageVector.begin() + counter, imageVector.end(), outputTicks[i]) -
            (imageVector.begin() + counter);

        counter += outputHist[i];
    }
    /// If you have a value of 100 as max value, the previous for loop will stop
    // just before the value 100. If several pixels have a value of 100, all of
    // them will not appear in the histogram. Therefore, we include them into the
    // last bin
    double remaining = imageVector.end() - (imageVector.begin() + counter);
    if (remaining > 0)
    {
        outputHist.back() += remaining;
    }
}

void MatPlotLibPlotsWidget::plotHistogram(cv::Mat data, int bins, double minX, double maxX, std::string title)
{
    std::vector<double> ticks;
    std::vector<double> hist;

    computeHistogram(data, bins, minX, maxX, ticks, hist);

    double lineWidth = ticks[1] - ticks[0];

    plt::figure();
    plt::bar(ticks, hist, lineWidth);
    std::map<std::string, std::string> fontsizeParam({std::pair<std::string, std::string>("fontsize", std::to_string(_plotFontSize))});
    std::map<std::string, std::string> labelsizeParam({std::pair<std::string, std::string>("labelsize", std::to_string(_plotLabelSize))});

    plt::title(title, fontsizeParam);
    plt::xlim(minX, maxX);
    plt::tick_params(labelsizeParam);
    plt::grid(true);
    plt::draw();
}

void MatPlotLibPlotsWidget::closePlots()
{
    _isLiveRowRunning->setChecked(false);
    plt::close();
}

void MatPlotLibPlotsWidget::createValueDistributionPlot(
    const std::vector<cv::Mat>& pixelParameterImage,
    const std::vector<std::string>& pixelIdsStrings,
    std::string paramType,
    double distMaxVal)
{
    assert(pixelParameterImage.size() == pixelIdsStrings.size());

    plt::close();

    for (unsigned int i = 0; i < pixelParameterImage.size(); i++)
    {
        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(pixelParameterImage[i], mean, stddev);

        std::string plotTitle = std::string(
            paramType +
            " distribution plot." +
            "\nMean: " + std::to_string(mean[0]) +
            " Std. dev.: " + std::to_string(stddev[0]) +
            "\nPixel: " + pixelIdsStrings[i]);

        double minval, maxval;
        cv::minMaxLoc(pixelParameterImage[i], &minval, &maxval);
        if (minval > 0)
        {
            minval = 0;
        }

        plotHistogram(pixelParameterImage[i],
            _histAmountBins,
            minval,
            distMaxVal,
            plotTitle);
    }
}

void MatPlotLibPlotsWidget::createSamplesPlot(const std::vector<double>& angles, const cv::Mat& avgImgs)
{
    int startRow = _pixelOffsetRows->value();
    int startColumn = _pixelOffsetCols->value();
    int numberRows = 4;
    int numberCols = 4;

    plt::close();

    std::vector<std::vector<double>> plotValues;

    if (angles.size() && !avgImgs.empty())
    {
        cv::Range rows(startRow, startRow + numberRows);
        cv::Range cols(startColumn, startColumn + numberCols);

        // We convert only the samples to the double type to be independent of
        // the input datatype
        cv::Mat samplesToPlot;
        avgImgs(rows, cols).convertTo(samplesToPlot, CV_64F);

        int channels = samplesToPlot.channels();

        for (int i = 0; i < samplesToPlot.rows; i++)
        {
            for (int j = 0; j < samplesToPlot.cols; j++)
            {
                cv::Mat pixelVal = cv::Mat(1, channels, CV_64FC1, samplesToPlot.ptr(i,j));
                plotValues.push_back(convertToVector<double>(pixelVal));
            }
        }

        double minX = *std::min_element(angles.begin(), angles.end());
        double maxX = *std::max_element(angles.begin(), angles.end());
        for (unsigned int i = 0; i < plotValues.size(); i++)
        {
            // We take 20% more than the maximum value, and 20% less for the minimum
            // value. Since we know the image values are always positive values,
            // these expressions are ok (they are not true if we have to consider
            // negative values)
            float minY = (*std::min_element(plotValues[i].begin(), plotValues[i].end())) * 0.8;
            float maxY = (*std::max_element(plotValues[i].begin(), plotValues[i].end())) * 1.2;

            std::map<std::string, std::string> fontsizeParam({std::pair<std::string, std::string>("fontsize", std::to_string(_plotFontSize))});
            std::map<std::string, std::string> labelsizeParam({std::pair<std::string, std::string>("labelsize", std::to_string(_plotLabelSize))});

            plt::figure();
            plt::plot(angles, plotValues[i]);
            plt::title(std::string("Plot for the pixel ") + std::to_string(i), fontsizeParam);
            plt::xlim(minX, maxX);
            plt::ylim(minY, maxY);
            plt::grid(true);
            plt::tick_params(labelsizeParam);
            plt::draw();
        }
    }
}

void MatPlotLibPlotsWidget::setEnableRunningState(bool enabled)
{
    _livePlotGroup->setEnabled(enabled);
    _samplesGroup->setEnabled(enabled);
    _distrGroup->setEnabled(enabled);
}

void MatPlotLibPlotsWidget::openFigures()
{
    plt::close();
    plt::ion();
    _intensityFigNum = plt::figure();
    plt::grid(true);
    _aolpFigNum = plt::figure();
    plt::grid(true);
    _dolpFigNum = plt::figure();
    plt::grid(true);
    plt::draw();
}

void MatPlotLibPlotsWidget::onSwitchRunningState(int state)
{
    bool enabled = state == Qt::Checked;
    emit switchRowPlot(enabled);
    setEnableRunningState(!enabled);

    plt::close();
    if (enabled)
    {
        openFigures();
    }
    else
    {
        _intensityFigNum = -1;
        _aolpFigNum = -1;
        _dolpFigNum = -1;
    }
}

void MatPlotLibPlotsWidget::plotLiveRow(std::vector<cv::Mat> polaImgs, ColorChannelType ch)
{
    if (_intensityFigNum != -1 && _aolpFigNum != -1 && _dolpFigNum != -1)
    {
        // 4 Channels (red, green_1, green_2 and blue) and 3 pola parameters (S0, AoLP, DoLP)
        assert(polaImgs.size() == 12);

        // The input vector has all the intensities together, all the AoLP together, and then all the DoLP together.
        std::vector<float> intVectorToPlot;
        std::vector<float> aolpVectorToPlot;
        std::vector<float> dolpVectorToPlot;

        int rowToExtract = polaImgs[0].rows * float(_rowToPlot->value()) / 100.0;
        polaImgs[ch].row(rowToExtract).copyTo(intVectorToPlot);
        cv::Mat aolpImg;
        polaImgs[ch + 4].row(rowToExtract).convertTo(aolpImg, CV_32FC1, 180.0 / _imgFormat->_maxAllowedVal);
        aolpImg.copyTo(aolpVectorToPlot);
        cv::Mat dolpImg;
        polaImgs[ch + 8].row(rowToExtract).convertTo(dolpImg, CV_32FC1, 1.0 / _imgFormat->_maxAllowedVal);
        dolpImg.copyTo(dolpVectorToPlot);

        // We create a vector with increasing values from 0 until cols - 1
        std::vector<float> x(intVectorToPlot.size(), 0);
        for (std::size_t i = 0; i < x.size(); i++)
        {
            x[i] = i;
        }

        float intMinVal = (*std::min_element(intVectorToPlot.begin(), intVectorToPlot.end())) * 0.8;
        float aolpMinVal = (*std::max_element(aolpVectorToPlot.begin(), aolpVectorToPlot.end())) * 1.2;
        float dolpMinVal = (*std::min_element(dolpVectorToPlot.begin(), dolpVectorToPlot.end())) * 0.8;
        float intMaxVal = (*std::max_element(intVectorToPlot.begin(), intVectorToPlot.end())) * 1.2;
        float aolpMaxVal = (*std::min_element(aolpVectorToPlot.begin(), aolpVectorToPlot.end())) * 0.8;
        float dolpMaxVal = (*std::max_element(dolpVectorToPlot.begin(), dolpVectorToPlot.end())) * 1.2;

        std::map<std::string, std::string> linewidthParam({std::pair<std::string, std::string>("linewidth", std::to_string(3))});
        std::map<std::string, std::string> fontsizeParam({std::pair<std::string, std::string>("fontsize", std::to_string(_plotFontSize))});
        std::map<std::string, std::string> labelsizeParam({std::pair<std::string, std::string>("labelsize", std::to_string(_plotLabelSize))});

        try
        {
            // Intensity plot
            plt::figure(_intensityFigNum);
            plt::clf();
            plt::plot(x, intVectorToPlot, linewidthParam);
            plt::title("Intensity plot - Row: " + std::to_string(rowToExtract), fontsizeParam);
            plt::tick_params(labelsizeParam);
            plt::ylim(intMinVal, intMaxVal);
            plt::grid(true);
            plt::draw();

            // AoLP plot
            plt::figure(_aolpFigNum);
            plt::clf();
            plt::plot(x, aolpVectorToPlot, linewidthParam);
            plt::title("AoLP plot - Row: " + std::to_string(rowToExtract), fontsizeParam);
            plt::tick_params(labelsizeParam);
            plt::ylim(aolpMinVal, aolpMaxVal);
            plt::grid(true);
            plt::draw();

            // DoLP plot
            plt::figure(_dolpFigNum);
            plt::clf();
            plt::plot(x, dolpVectorToPlot, linewidthParam);
            plt::title("DoLP plot - Row: " + std::to_string(rowToExtract), fontsizeParam);
            plt::tick_params(labelsizeParam);
            plt::ylim(dolpMinVal, dolpMaxVal);
            plt::grid(true);
            plt::draw();
        }
        catch (const std::runtime_error& e)
        {
            std::cout << "Runtime error happened. Re-opening the windows" << std::endl;
            std::cout << e.what() << std::endl;
            openFigures();
        }
    }
}

void MatPlotLibPlotsWidget::onRequestDistributionData()
{
    emit requestDistributionData(_chosenDistPlot);
}

void MatPlotLibPlotsWidget::onRequestLiveDistributionPlot()
{
    emit requestLiveDistributionPlot(_chosenLivePlot);
}

void MatPlotLibPlotsWidget::onGainPlotSelected(bool state)
{
    if (state)
    {
        _chosenDistPlot = GAIN_PLOT;
        _distributionPlotMaxVal = DISTRIBUTION_GAIN_MAX_VAL;
    }
}

void MatPlotLibPlotsWidget::onPhasePlotSelected(bool state)
{
    if (state)
    {
        _chosenDistPlot = PHASE_PLOT;
        _distributionPlotMaxVal = DISTRIBUTION_PHASE_MAX_VAL;
    }
}

void MatPlotLibPlotsWidget::onTiPlotSelected(bool state)
{
    if (state)
    {
        _chosenDistPlot = TI_PLOT;
        _distributionPlotMaxVal = DISTRIBUTION_NON_IDEALITY_MAX_VAL;
    }
}

void MatPlotLibPlotsWidget::onPiPlotSelected(bool state)
{
    if (state)
    {
        _chosenDistPlot = PI_PLOT;
        _distributionPlotMaxVal = DISTRIBUTION_NON_IDEALITY_MAX_VAL;
    }
}

void MatPlotLibPlotsWidget::onBlackCurrentPlotSelected(bool state)
{
    if (state)
    {
        _chosenDistPlot = BLACK_CURRENT_PLOT;
        _distributionPlotMaxVal = DISTRIBUTION_NON_IDEALITY_MAX_VAL;
    }
}

void MatPlotLibPlotsWidget::onAoPLivePlotRequested(bool state)
{
    if (state)
    {
        _liveDistMaxVal = _imgFormat->_maxAllowedVal;
        _chosenLivePlot = AOP_PLOT;
    }
}

void MatPlotLibPlotsWidget::onDoPLivePlotRequested(bool state)
{
    if (state)
    {
        _liveDistMaxVal = _imgFormat->_maxAllowedVal;
        _chosenLivePlot = DOP_PLOT;
    }
}

void MatPlotLibPlotsWidget::onIntensityLivePlotRequested(bool state)
{
    if (state)
    {
        _liveDistMaxVal = _imgFormat->_maxAllowedVal;
        _chosenLivePlot = INTENSITY_PLOT;
    }
}
