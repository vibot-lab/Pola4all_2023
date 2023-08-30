// OpenCV includes

// STD includes
#include <fstream>
#include <iomanip>      // For std::setprecision
#include <iostream>
#include <map>
#include <sstream>

// Qt Includes
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFont>
#include <QInputDialog>
#include <QMessageBox>
#include <QStringList>
#include <QThread>

// Custom includes
#include "mainwindow.h"
#include "PolarimetricCamera/PolarimetricImagesProcessing/PolarimetricImagesProcessing.hpp"
#include "ui_mainwindow.h"

/**
 * @brief printVector: Print the given vector in the form:
 *                  name = [e0, e1, e2, ... eN]
*/
template<typename T>
void printVector(std::vector<T> vec, std::string name)
{
    std::cout << name << " = [" << vec[0];
    for (size_t i = 1; i < vec.size(); i++)
    {
        std::cout << ", " << vec[i];
    }
    std::cout << "]" << std::endl;
}

MainWindow::MainWindow(std::shared_ptr<bitDepthStruct> camImgFormat, QWidget *parent)
    : QMainWindow(parent),
      _leftLayout(new QVBoxLayout()),
      _rightLayout(new QVBoxLayout()),
      _mainLayout(new QHBoxLayout()),
      _imageWidget(nullptr),
      _loadSaveExpWidget(nullptr),
      _matPlotLibWidget(nullptr),
      _paramsWidget(nullptr),
      _saveImgWidget(nullptr),
      _messageLabel(nullptr),
      _tempLabel(nullptr),
      _scroll(nullptr),
      _connect(nullptr),
      _disconnect(nullptr),
      _exit(nullptr),
      _refreshCameraPeriod(40),
      _refreshRTParamsPeriod(500),
      _useCalibrationCheckBox(nullptr),
      _useSuperPixelCheckBox(nullptr),
      _imageWidth(nullptr),
      _tabs(nullptr),
      _camera(nullptr),
      ui(new Ui::MainWindow),
      _experimentsSaverLoader(camImgFormat),
      _avgAngleEstimation({0, 1, 2, 3}, 50),
      _liveParamsWidget(nullptr),
      _modeWidget(nullptr),
      _isRunningRowPlotMode(false),
      _camImgFormat(camImgFormat)
{
    ui->setupUi(this);

    _tabs = new QTabWidget(this);

    initializeWidgets();

    connect(&_refreshDataTimer,
        &QTimer::timeout,
        this,
        &MainWindow::refreshCameraData);

    connect(&_refreshRealTimeParamsTimer,
        &QTimer::timeout,
        this,
        &MainWindow::refreshRealTimeParams);

    _refreshDataTimer.stop();
    _refreshRealTimeParamsTimer.stop();

    resetGuiState();
    onUpdateFilterOrientations(_filtersConfig.getPixelsMap());
    setWindowTitle("Pola4All");
}

MainWindow::~MainWindow()
{
    onDisconnectCamera();
    delete ui;
    delete _leftLayout;
    delete _rightLayout;
    delete _mainLayout;
    delete _imageWidget;
    delete _loadSaveExpWidget;
    delete _paramsWidget;
    delete _saveImgWidget;
    delete _messageLabel;
    delete _tempLabel;
    delete _scroll;
    delete _connect;
    delete _disconnect;
    delete _matPlotLibWidget;
    delete _exit;
    delete _imageWidth;
    delete _useCalibrationCheckBox;
    delete _useSuperPixelCheckBox;
    delete _tabs;

    // delete _modeWidget;
    // delete _liveParamsWidget;
}

void MainWindow::initializeWidgets()
{
    addButtons();
    addCalibrationCheckBox();
    addImageWidget();
    QWidget* tabOneWidget = createTabOne();
    _tabs->addTab(tabOneWidget, "Camera control");
    QWidget* tabTwoWidget = createTabTwo();
    _tabs->addTab(tabTwoWidget, "Camera calibration");
    QWidget* tabThreeWidget = createTabThree();
    _tabs->addTab(tabThreeWidget, "Plotting");

    _rightLayout->addWidget(_tabs);

    addExitButton();

    connect(this,
        &MainWindow::updateState,
        this,
        &MainWindow::onUpdateState);

    // All the desired widgets must be placed before this point
    initializeMainWidowLayouts();
    setButtonsEnabled(false);

    updateCurrentLabels();
}

void MainWindow::initializeMainWidowLayouts()
{
    // We create the layouts that are going to host the left and right layouts
    QWidget *leftWidget = new QWidget();
    QWidget *rightWidget = new QWidget();
    leftWidget->setLayout(_leftLayout);

    // This line is really important. It avoids the right layout to expand
    // without limits. With this line, we avoid that, and we leave more
    // space for the images
    rightWidget->setLayout(_rightLayout);
    // This is the line that makes the right widget to take small space
    // horizontally, and expanding vertically
    rightWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

    // We add the left and right layouts to the main one
    _mainLayout->addWidget(leftWidget);
    _mainLayout->addWidget(rightWidget);
    QWidget* internalWidget = new QWidget();

    internalWidget->setLayout(_mainLayout);
    setCentralWidget(internalWidget);
}

QWidget* MainWindow::createTabOne()
{
    QScrollArea* scrolling = new QScrollArea(this);
    QWidget* tabWidget = new QWidget(scrolling);

    QVBoxLayout* localLayout = new QVBoxLayout();

    initializeTemperatureLabel(localLayout, tabWidget);
    addImageWidthSpinBox(localLayout, tabWidget);
    addParametersWidget(localLayout, tabWidget);
    addModeWidget(localLayout, tabWidget);
    addSaveImagesWidget(localLayout, tabWidget);

    tabWidget->setLayout(localLayout);

    scrolling->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrolling->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrolling->setWidgetResizable(true);
    scrolling->setWidget(tabWidget);

    return scrolling;
}

std::string MainWindow::createStringWithNumber(std::string txt, double number, int precision)
{
    std::ostringstream sstream;
    sstream << txt  << std::fixed << std::setprecision(precision) << number;
    return sstream.str();
}

void MainWindow::addParametersShowingWidget(QLayout* localLayout, QWidget* parent)
{
    _liveParamsWidget = new RealTimeParameters(_camImgFormat, parent);
    localLayout->addWidget(_liveParamsWidget);
}

QWidget* MainWindow::createTabTwo()
{
    QScrollArea* scrolling = new QScrollArea(this);
    QWidget* tabWidget = new QWidget(scrolling);
    QVBoxLayout* localLayout = new QVBoxLayout();

    addPixelCalibrationWidget(localLayout, tabWidget);
    addParametersShowingWidget(localLayout, tabWidget);
    addAoLPEstimatorTestButton(localLayout, tabWidget);
    localLayout->addStretch(1);

    tabWidget->setLayout(localLayout);

    scrolling->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrolling->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrolling->setWidgetResizable(true);
    scrolling->setWidget(tabWidget);

    return scrolling;
}

QWidget* MainWindow::createTabThree()
{
    QScrollArea* scrolling = new QScrollArea(this);
    QWidget* tabWidget = new QWidget(scrolling);
    QVBoxLayout* localLayout = new QVBoxLayout();

    addPlotsWidget(localLayout, tabWidget);
    tabWidget->setLayout(localLayout);

    scrolling->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrolling->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrolling->setWidgetResizable(true);
    scrolling->setWidget(tabWidget);

    return scrolling;
}

void MainWindow::addImageWidget()
{
    _messageLabel = new QLabel("Camera not connected", this);
    QFont f("Arial", 14, QFont::Bold, true);
    _messageLabel->setStyleSheet("QLabel { color : blue; }");

    _messageLabel ->setFont(f);
    _imageWidget = new ImageGridWidget(this);

    _scroll = new QScrollArea(this);
    _scroll->setWidget(_imageWidget);
    _scroll->setWidgetResizable(true);

    _leftLayout->addWidget(_messageLabel);
    _leftLayout->addWidget(_scroll);
}

void MainWindow::initializeTemperatureLabel(QLayout* layout, QWidget* parent)
{
    _tempLabel = new QLabel("", parent);
    _tempLabel->setAlignment(Qt::AlignHCenter);
    QFont f("Arial", 14, QFont::Bold, true);
    _tempLabel->setFont(f);

    layout->addWidget(_tempLabel);
}

void MainWindow::addImageWidthSpinBox(QLayout* layout, QWidget* parent)
{
    QLabel* imageWidthLabel = new QLabel("Image width:", parent);

    _imageWidth = new QSpinBox(parent);
    /// OpenCV crashes if we try to resize the image to have one pixel, so we
    // limit theminimum spinbox value to be 2.
    _imageWidth->setMinimum(2);
    /// We do not know why, but the program crashes when the size of the image
    // becomes large. The ImageWidget paintEvent cannot draw it, so we limit
    // the maximum value to 3000 to avoid problems.
    _imageWidth->setMaximum(3000);
    _imageWidth->setSingleStep(10);
    // We set the initial SpinBox value with the default image width
    int defaultImageWidth = _imageWidget->getImageWidth();
    if (defaultImageWidth != -1)
    {
        _imageWidth->setValue(defaultImageWidth);
    }

    QFormLayout* tempLayout = new QFormLayout();
    QWidget* tempWidget = new QWidget(parent);
    tempLayout->addRow(imageWidthLabel, _imageWidth);
    tempWidget->setLayout(tempLayout);

    layout->addWidget(tempWidget);

    connect(_imageWidth,
        static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
        this,
        &MainWindow::onImageWidthChanged);
}

void MainWindow::addButtons()
{
    _connect = new QPushButton("Connect to the camera", this);
    _disconnect = new QPushButton("Disconnect", this);
    QPushButton* changeFilters = new QPushButton("Change filters configuration", this);

    _rightLayout->addWidget(_connect);
    _rightLayout->addWidget(_disconnect);
    _rightLayout->addWidget(changeFilters);

    connect(_connect, &QPushButton::clicked, this, &MainWindow::onConnectCamera);
    connect(_disconnect, &QPushButton::clicked, this, &MainWindow::onDisconnectCamera);
    connect(changeFilters, &QPushButton::clicked, &_filtersConfig, &FilterOrientationsWidget::showDialog);
    connect(&_filtersConfig, &FilterOrientationsWidget::orientationsUpdated, this, &MainWindow::onUpdateFilterOrientations);
}

void MainWindow::addCalibrationCheckBox()
{
    _useCalibrationCheckBox = new QCheckBox("Correct pixel gain", this);
    _useSuperPixelCheckBox = new QCheckBox("Use SuperPixel calibration method", this);
    _useSuperPixelCheckBox->setChecked(true);
    _rightLayout->addWidget(_useCalibrationCheckBox);
    _rightLayout->addWidget(_useSuperPixelCheckBox);
}

void MainWindow::addParametersWidget(QLayout* layout, QWidget* parent)
{
    _paramsWidget = new ParametersWidget({0, 24}, {21, 1e+7}, parent);
    layout->addWidget(_paramsWidget);

    connect(_paramsWidget,
        &ParametersWidget::updateParameters,
        this,
        &MainWindow::updateCameraParameters);
}

void MainWindow::addModeWidget(QLayout* layout, QWidget* parent)
{
    _modeWidget = new VisualizationModeWidget(parent);
    layout->addWidget(_modeWidget);

    connect(
        _modeWidget,
        &VisualizationModeWidget::modeChanged,
        this,
        &MainWindow::updateCurrentLabels);
}

void MainWindow::addSaveImagesWidget(QLayout* layout, QWidget* parent)
{
    _saveImgWidget = new SaveImagesWidget(parent);
    layout->addWidget(_saveImgWidget);

    connect(_saveImgWidget,
        &SaveImagesWidget::requestCaptureImages,
        this,
        &MainWindow::onCaptureImageRequest);
}

void MainWindow::addExitButton()
{
    _exit = new QPushButton("Exit", this);
    _rightLayout->addWidget(_exit);
    connect(_exit, &QPushButton::clicked, this, &MainWindow::onExit);
}

void MainWindow::addAoLPEstimatorTestButton(QLayout* layout, QWidget* parent)
{
    QPushButton* testDiffROIAoLPEstButton = new QPushButton("AoLP estimator ROI test", parent);

    layout->addWidget(testDiffROIAoLPEstButton);

    connect(testDiffROIAoLPEstButton,
        &QPushButton::clicked,
        this,
        &MainWindow::onTestDifferentROI);
}

void MainWindow::addPixelCalibrationWidget(QLayout* layout, QWidget* parent)
{
    _loadSaveExpWidget = new LoadSaveWidget(_pixelGainCalibratorHub.getListCalibrators(), parent);

    layout->addWidget(_loadSaveExpWidget);

    connect(_loadSaveExpWidget,
        &LoadSaveWidget::loadFiles,
        this,
        &MainWindow::onLoadExperimentRequest);

    connect(_loadSaveExpWidget,
        &LoadSaveWidget::createExperiment,
        this,
        &MainWindow::onCreateNewExperiment);

    connect(_loadSaveExpWidget,
        &LoadSaveWidget::computeCalibration,
        this,
        &MainWindow::onCalibratePixels);

    connect(_loadSaveExpWidget,
        &LoadSaveWidget::loaderChanged,
        this,
        &MainWindow::updateMatPlotLibSamplesLimit);
}

void MainWindow::addPlotsWidget(QLayout* layout, QWidget* parent)
{
    _matPlotLibWidget = new MatPlotLibPlotsWidget(10, 10, _camImgFormat, parent);
    layout->addWidget(_matPlotLibWidget);

    connect(_matPlotLibWidget,
        &MatPlotLibPlotsWidget::requestDistributionData,
        this,
        &MainWindow::onPlotDistribution);

    connect(_matPlotLibWidget,
        &MatPlotLibPlotsWidget::requestSamples,
        this,
        &MainWindow::onPlotSamples);

    connect(_matPlotLibWidget,
        &MatPlotLibPlotsWidget::requestLiveDistributionPlot,
        this,
        &MainWindow::onLiveDistPlot);

    connect(_matPlotLibWidget,
        &MatPlotLibPlotsWidget::switchRowPlot,
        this,
        &MainWindow::onLiveRowPlotStateChanged);
}

void MainWindow::setButtonsEnabled(bool enabled)
{
    _connect->setEnabled(!enabled);
    _disconnect->setEnabled(enabled);
    _paramsWidget->setEnabled(enabled);
    _saveImgWidget->setEnabled(enabled);
    _imageWidth->setEnabled(enabled);
    _loadSaveExpWidget->setEnabled(enabled);
    _matPlotLibWidget->setEnabled(enabled);
    _modeWidget->setEnabled(enabled);
}

void MainWindow::onConnectCamera()
{
    _camera = &RosCamClient::getInstance();
    if (_camera)
    {
        /// WARNING: The callback provided here might be called from another
        // thread that is not the main!
        _camera->setStateCallback(this);
    }
    else
    {
        QMessageBox::critical(this,
            "Connecting to the camera",
            "Cannot retrieve the ROS client object! Aborting",
            QMessageBox::Ok);
        return;
    }

    if (_camera->initCam(_camImgFormat->_bitDepth) && _camera->isAlive())
    {
        _paramsWidget->setGainLimits(_camera->getGainLimits());
        _paramsWidget->setExposureLimits(_camera->getExposureLimits());
        _messageLabel->setText("Camera connected");
        _messageLabel->setStyleSheet("QLabel { color : green; }");

        // Software White Balance
        _whiteBalanceControl.enableAutoWhiteBalance(_whiteBalanceControl.isAutoWhiteBalanceEnabled());
        _paramsWidget->setAutoWhiteBalanceState(_whiteBalanceControl.isAutoWhiteBalanceEnabled());
        _paramsWidget->setSoftwareWhiteBalanceGains(_whiteBalanceControl.getGains());

        _refreshDataTimer.start(_refreshCameraPeriod);
        _refreshRealTimeParamsTimer.start(_refreshRTParamsPeriod);

        setButtonsEnabled(true);
    }
    else
    {
        resetGuiState();
    }
}

void MainWindow::resetGuiState()
{
    _tempLabel->setText("Temperature (°C): 0.00");
    setButtonsEnabled(false);
    _messageLabel->setText("Camera not connected");
    _messageLabel->setStyleSheet("QLabel { color : blue; }");
}

void MainWindow::onDisconnectCamera()
{
    _refreshDataTimer.stop();
    _refreshRealTimeParamsTimer.stop();

    if (_camera)
    {
        _camera->deinitCam();
    }
    _imageWidget->clearBuffers();
    resetGuiState();
}

void MainWindow::onExit()
{
    onDisconnectCamera();
    qApp->quit();
}

void MainWindow::onCaptureImageRequest()
{
    assert(_camera);

    updateCurrentLabels();

    std::vector<cv::Mat> imgs;
    cv::Mat raw;
    _camera->getSingleImageSynchronously(raw);

    bool useCalib = _useCalibrationCheckBox->checkState() == Qt::Checked;
    cv::Mat correctedImg;
    _pixelGainCalibratorHub.correctImage(raw, useCalib, correctedImg);

    ProcessingParameters params = {
        .action = _modeWidget->getSelectedMode(),
        .aop = _modeWidget->getAoLP(),
        .filtersMap = _filtersConfig.getPixelsMap(),
        .imgFormatData = _camImgFormat
    };
    PolarimetricImagesProcessing::processImage(correctedImg, params, imgs);

    // Software white balance
    imgs = _whiteBalanceControl.applyBalance(imgs);

    _saveImgWidget->onImagesReceived(imgs);
}

void MainWindow::computeAoLPTestMetrics(const std::vector<double> &estAngles, const std::vector<double>& gtAngles, double &maxVal, double &rmseVal)
{
    rmseVal = 0;
    maxVal = 0;
    if (estAngles.size() && estAngles.size() == gtAngles.size())
    {
        double firstEst = estAngles[0];
        for(size_t i = 0; i < estAngles.size(); i++)
        {
            double err = std::abs((estAngles[i] - firstEst) - gtAngles[i]);
            if (err > maxVal)
            {
                maxVal = err;
            }
            rmseVal += (err * err);
        }
        rmseVal = std::sqrt(rmseVal / estAngles.size());
    }
    else
    {
        std::cout << "ERROR: The GT and the estimations vectors do not have the same size!" << std::endl;
    }
}

void MainWindow::onUpdateFilterOrientations(std::map<int,int> map)
{
    assert(map.size() == 4);
    // This vector will have the filter orientations (measured in degrees)
    // in the right order.
    std::vector<int> orderedOrientations(4);
    orderedOrientations[map[135]] = 135;
    orderedOrientations[map[90]] = 90;
    orderedOrientations[map[45]] = 45;
    orderedOrientations[map[0]] = 0;

    // We update the orientations positions for the AoLP estimator
    _avgAngleEstimation.updateSPOrientations(std::vector<int>({map[135], map[0], map[90], map[45]}));

    // We update the orientations used for the uncalibration matrix
    _pixelGainCalibratorHub.updateDefaultFilterOrientations(orderedOrientations);

    // We update the current labels shown along with the images
    _modeWidget->updateAngleLabels(orderedOrientations);
    updateCurrentLabels();
}

void MainWindow::runROIChangeErrorInAoLP()
{
    std::string calibrator = _pixelGainCalibratorHub.getGainCalibName();
    const cv::Mat& imgs = _pixelGainCalibratorHub.getRawSamples(calibrator);
    const std::vector<double>& gtAngles = _experimentsSaverLoader.getAngles();

    int maxROIwidth = imgs.cols / 2;
    int maxROIheight = imgs.rows / 2;
    int maxROI = (maxROIwidth > maxROIheight ? maxROIheight : maxROIwidth);

    std::vector<double> maxValues;
    std::vector<double> rmseValues;
    std::vector<double> roiValues;
    std::map<int, int> map = _filtersConfig.getPixelsMap();

    /// For each region size, we first compute all the estimated
    // angles, and then we compute the metrics.
    for (int roiSize = 0; roiSize < maxROI; roiSize += 2)
    {
        std::cout << "Testing for ROI size = " << roiSize << std::endl;
        SourceOrientationEstimator estimator(std::vector<int>({map[135], map[0], map[90], map[45]}), roiSize);
        std::vector<double> estAngles;

        for (int i = 0; i < imgs.channels(); i++)
        {
            cv::Mat sample;
            cv::extractChannel(imgs, sample, i);
            estAngles.push_back(estimator.getLightSourceOrientation(sample));
        }

        double maxVal;
        double rmseVal;
        computeAoLPTestMetrics(estAngles, gtAngles, maxVal, rmseVal);

        maxValues.push_back(maxVal);
        rmseValues.push_back(rmseVal);
        roiValues.push_back(roiSize);
    }
    printVector(roiValues, "roi_val_vector");
    printVector(maxValues, "max_val_vector");
    printVector(rmseValues, "rmse_val_vector");
}

void MainWindow::onTestDifferentROI()
{
    const std::vector<double>& gtAngles = _experimentsSaverLoader.getAngles();
    if (gtAngles.size())
    {
        executeHeavyTasks(
            &MainWindow::runROIChangeErrorInAoLP,
            "Testing AoLP estimator",
            "Testing the AoLP estimator for different sizes of the ROI. Please, wait..."
        );
    }
    else
    {
        std::cout << "To run this test you need first to load / create an experiment" << std::endl;
    }
}

void MainWindow::updateCurrentLabels()
{
    _imageWidget->setImageLabels(_modeWidget->getSelectedLabels());
    _saveImgWidget->setImageLabels(_modeWidget->getSelectedLabels());
}

void MainWindow::updateMatPlotLibSamplesLimit()
{
    std::string calibrator = _loadSaveExpWidget->getSelectedKey();
    const cv::Mat& imgSamples = _pixelGainCalibratorHub.getRawSamples(calibrator);
    if (!imgSamples.empty())
    {
        _matPlotLibWidget->updateImageSize(imgSamples.rows, imgSamples.cols);
    }
    else
    {
        _matPlotLibWidget->updateImageSize(0, 0);
    }
}

void MainWindow::updateStateCallback(const CameraState &newState)
{
    /// I know this implementation looks stupid, but there is not another way
    // if the updateStateCallback is called from another thread that is not
    // the main.
    if (_paramsWidget)
    {
        emit updateState(newState);
    }
}

void MainWindow::onImageWidthChanged(int value)
{
    // When the spinbox value changes, we update the image width value
    _imageWidget->changeImageWidth(value);
}

void MainWindow::onUpdateState(CameraState newState)
{
    if (_paramsWidget)
    {
        _paramsWidget->onNewStateReceived(newState);
        // Software white balance
        _paramsWidget->setAutoWhiteBalanceState(_whiteBalanceControl.isAutoWhiteBalanceEnabled());
        _paramsWidget->setSoftwareWhiteBalanceGains(_whiteBalanceControl.getGains());
    }
}

void MainWindow::updateCameraParameters(CameraParams paramToChange, CameraState state)
{
    assert(_camera);
    switch(paramToChange)
    {
        case WHITE_BALANCE_GAINS:
        {
            _whiteBalanceControl.setGains(std::vector<double>({state.redGain, state.greenGain, state.blueGain}));
            break;
        }
        case AUTO_WHITE_BALANCE:
        {
            _whiteBalanceControl.enableAutoWhiteBalance(state.autoWhiteBalanceEnabled);
            if (!state.autoWhiteBalanceEnabled)
            {
                _paramsWidget->setSoftwareWhiteBalanceGains(_whiteBalanceControl.getGains());
            }
            break;
        }
        default:
        {
            _camera->changeCameraParameters(paramToChange, state);
            break;
        }
    }
}

void MainWindow::refreshRealTimeParams()
{
    cv::Mat rawImg;
    _camera->getRawImage(rawImg);
    if (_liveParamsWidget->isEnabled() && (!rawImg.empty()))
    {
        std::vector<cv::Mat> imgs;
        ProcessingParameters params = {
            .action = RAW_I_RO_PHI,
            .aop = -1,
            .filtersMap = _filtersConfig.getPixelsMap(),
            .imgFormatData = _camImgFormat
        };
        PolarimetricImagesProcessing::processImage(rawImg, params, imgs);

        ParametersSet liveParams;
        cv::minMaxLoc(rawImg, &liveParams.minInt, &liveParams.maxInt);

        cv::Scalar meanDoLP;
        cv::Mat mergedDoLP;
        // We take the four channels of the DoLP from the imgs vector, and we
        // mix them into a single image.
        std::vector<cv::Mat> doLPChannels(imgs.begin() + 8, imgs.end());
        cv::merge(doLPChannels, mergedDoLP);
        // cv::mean gives you the mean per channels. Therefore, the output has
        // 4 elements in this case
        meanDoLP = cv::mean(mergedDoLP);
        liveParams.meanDoLP = (meanDoLP[0] + meanDoLP[1] + meanDoLP[2] + meanDoLP[3]) / 4.0;
        // We convert the four channeled image into an image with a single
        // channel image, with the same amount of rows than one of the channels.
        cv::Mat singleChannelMat = mergedDoLP.reshape(1);
        cv::minMaxLoc(singleChannelMat, &liveParams.minDoLP, &liveParams.maxDoLP);

        // Since the circular average is already computed fast with
        // the average angle estimator, we use it. Sadly, this does not
        // considers the calibrated matrix.
        liveParams.aolp = _avgAngleEstimation.getLightSourceOrientation(rawImg);

        // We update the labels.
        _liveParamsWidget->updateParamLabels(liveParams);
    }
}

void MainWindow::refreshCameraData()
{
    assert(_camera);
    /// We update the shown image
    // If no image is received, it is because the camera got disconnected
    cv::Mat raw;
    _camera->getRawImage(raw);

    if (!raw.empty())
    {
        cv::Mat correctedImg;
        bool useCalib = _useCalibrationCheckBox->checkState() == Qt::Checked;
        _pixelGainCalibratorHub.correctImage(raw, useCalib, correctedImg);
        std::vector<cv::Mat> imgs;
        ProcessingParameters params = {
            .action = _modeWidget->getSelectedMode(),
            .aop = _modeWidget->getAoLP(),
            .filtersMap = _filtersConfig.getPixelsMap(),
            .imgFormatData = _camImgFormat
        };

        if (_isRunningRowPlotMode)
        {
            params.action = RAW_I_RO_PHI;
            PolarimetricImagesProcessing::processImage(correctedImg, params, imgs);
            _matPlotLibWidget->plotLiveRow(imgs, RED_CHANNEL);
        }
        else
        {
            PolarimetricImagesProcessing::processImage(correctedImg, params, imgs);

            // We apply the software white balance
            imgs = _whiteBalanceControl.applyBalance(imgs);

            int amountCols = 2;
            if (_modeWidget->getSelectedMode() >= STOKES)
            {
                amountCols = 4;
            }
            updateCurrentLabels();
            _imageWidget->updateGridImages(imgs, amountCols);
        }
    }

    /// We update the temperature label
    _tempLabel->setText(createStringWithNumber("Temperature (°C): ", _camera->getTemperature(), 2).c_str());
}

void MainWindow::onLoadExperimentRequest(std::string expFolder, std::string expName)
{
    _expFolder = expFolder;
    _expName = expName;
    executeHeavyTasks(
        &MainWindow::loadExperimentData,
        "Loading data",
        "The data is being loaded. This operation can be long. Please, wait..."
    );
    // We clear the strings, to avoid loading an old experiment (should never happen)
    _expFolder = "";
    _expName = "";
}

void MainWindow::loadExperimentData()
{
    if (!_expFolder.empty() && !_expName.empty())
    {
        std::map<int, int> map = _filtersConfig.getPixelsMap();
        _experimentsSaverLoader.loadExperiment(_expFolder, _expName);
        _pixelGainCalibratorHub.loadData(_loadSaveExpWidget->getSelectedKey(),
            {map[135], map[0], map[90], map[45]},
            _experimentsSaverLoader.getAngles(),
            _experimentsSaverLoader.getImages());

        _experimentsSaverLoader.reset();
        std::cout << "Data loaded correctly" << std::endl;

        updateMatPlotLibSamplesLimit();
    }
    else
    {
        std::cout << "No experiment path nor experiment name loaded" << std::endl;
    }
}

void MainWindow::runPixelsCalibrations()
{
    bool useSP = _useSuperPixelCheckBox->checkState() == Qt::Checked;
    _pixelGainCalibratorHub.computeCalibrationMatrices(useSP);
    std::cout << "Calibration finished" << std::endl;
}

void MainWindow::onCalibratePixels()
{
    executeHeavyTasks(
        &MainWindow::runPixelsCalibrations,
        "Pixel gain calibration",
        "Computing the calibration matrices. This operation can be long. Please, wait..."
    );
}

void MainWindow::onCreateGainData(int samples, int angleStep, std::string path, std::string expName)
{
    /// We initialize the SaveLoad module (not the widget). This module will
    // initialize the internal variables, and do the running average of the samples.
    if (_experimentsSaverLoader.startExperiment(path, expName))
    {
        // We create a string with all the experiment description.
        QString description = "Now, we are going to take a series of pictures from the camera.\n";
        description += QString("For this, at each iteration, we will take %1 pictures, and we will do the average of them, pixel-wise.\n").arg(samples);
        description += "Each capture is generated by placing a linearly polarized light at certain angle in front of the camera.\n";
        description += "Each capture represents a value from a cosine curve. The more points we have, the higher the resolution of this curve will be.\n";
        description += QString("The captured images will be stored in disk in the path %1, in the directory %2.\n").arg(path.c_str()).arg(expName.c_str());
        description += "At any time the user can decide when to stop capturing images.\n";
        description += QString("The steps must be done every %1 degrees. Let's get started!.\n").arg(angleStep);

        // We create a QMessageBox with the description written below, to inform
        // what to do in the experiment.
        QMessageBox::information(this,
            "Pixel gains experiment",
            description,
            QMessageBox::Ok);

        // This message box will be shown each time we change the polarization angle.
        // Each time this message appears, the user can stop the experiment.
        QMessageBox grabImageBox;
        grabImageBox.setIcon(QMessageBox::Information);
        grabImageBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        QAbstractButton* endButton = grabImageBox.button(QMessageBox::Cancel);
        endButton->setText("Finish test");

        int angle = 0;

        grabImageBox.setText(QString("Press Ok when you are ready to grab a image at %1 degrees").arg(angle));
        // We iterate until the user decides to stop, or the camera gets disconnected.
        while(_camera->isAlive() && grabImageBox.exec() == QMessageBox::Ok)
        {
            QCoreApplication::processEvents();
            // We take as many samples as the user wants, for the same polarization
            // angle.
            for (int i = 0; i < samples; i++)
            {
                QCoreApplication::processEvents();
                // We ensure the read image is not empty
                cv::Mat raw;
                while (raw.empty())
                {
                    _camera->getSingleImageSynchronously(raw);
                    QCoreApplication::processEvents();
                }
                cv::Mat correctedImg;
                bool useCalib = _useCalibrationCheckBox->checkState() == Qt::Checked;
                _pixelGainCalibratorHub.correctImage(raw, useCalib, correctedImg);
                _experimentsSaverLoader.addExperimentSample(correctedImg, angle, true);
            }
            angle += angleStep;
            grabImageBox.setText(QString("Press Ok when you are ready to grab a image at %1 degrees").arg(angle));
        }

        _experimentsSaverLoader.endExperiment();

        std::map<int, int> map = _filtersConfig.getPixelsMap();
        _pixelGainCalibratorHub.loadData(
            _loadSaveExpWidget->getSelectedKey(),
            {map[135], map[0], map[90], map[45]},
            _experimentsSaverLoader.getAngles(),
            _experimentsSaverLoader.getImages());
        _experimentsSaverLoader.reset();

    }
    else
    {
        QMessageBox errorMessageBox;
        errorMessageBox.setIcon(QMessageBox::Warning);
        errorMessageBox.setStandardButtons(QMessageBox::Ok);
        QString description("ERROR! We cannot start the experiment. Check that\n");
        description += "there is not already an experiment with the same name,\n";
        description += "and that the working directory is writable.";
        errorMessageBox.setText(description);
        errorMessageBox.exec();
    }
}

void MainWindow::onCreateBlackCurrentData(int samples, int expTimeStep, std::string path, std::string expName)
{
    /// We initialize the SaveLoad module (not the widget). This module will
    // initialize the internal variables, and do the running average of the samples.
    if (_experimentsSaverLoader.startExperiment(path, expName))
    {
        blockSignals(true);
        int initialExpTime_uS = 100;
        int finalExpTime_uS = 2000;

        // We create a string with all the experiment description.
        QString description = "Now, we are going to take a series of pictures from the camera.\n";
        description += QString("For this, at each iteration, we will take %1 pictures, and we will do the average of them, pixel-wise.\n").arg(samples);
        description += "In all the cases, the camera must have the cover placed on, and at each iteration, the exposure time will be changed.\n";
        description += QString("The captured images will be stored in disk in the path %1, in the directory %2.\n").arg(path.c_str()).arg(expName.c_str());
        description += "At any time the user can decide when to stop capturing images.\n";
        description += QString("The exposure time will be done every %1 uS. Let's get started!.\n").arg(expTimeStep);
        description += QString("The exposure time will be tested in the range between %1 and %2 uS.\n").arg(initialExpTime_uS).arg(finalExpTime_uS);

        // We create a QMessageBox with the description written below, to inform
        // what to do in the experiment.
        QMessageBox::information(this,
            "Pixel black current experiment",
            description,
            QMessageBox::Ok);

        // We create a QMessageBox to allow the user to indicate when it is ready to start
        QMessageBox inProgressMsgBox;
        inProgressMsgBox.setIcon(QMessageBox::Information);
        inProgressMsgBox.setStandardButtons(QMessageBox::Cancel);
        QAbstractButton* endButton = inProgressMsgBox.button(QMessageBox::Cancel);
        inProgressMsgBox.setText("Press Abort to stop the experiment");
        endButton->setText("Abort");
        inProgressMsgBox.show();

        // We backup the camera state
        CameraState oldState = _camera->getCameraState();

        // We set a new state. We disable all the auto features, we set the gains to zero, and we
        // set the frame rate to 30 fps
        CameraState newState = {
            .gain = 0,
            .exposureUs = static_cast<double>(initialExpTime_uS),
            .frameRate = 30,
            .redGain = 1,
            .greenGain = 1,
            .blueGain = 1,
            .autoGainEnabled = false,
            .autoExposureEnabled = false,
            .autoWhiteBalanceEnabled = false,
        };
        _camera->changeCameraParameters(ALL, newState);

        // We iterate for all the exposure times, or the camera gets disconnected.
        for(int expTime = initialExpTime_uS; expTime <= finalExpTime_uS && _camera->isAlive() && inProgressMsgBox.result() != QMessageBox::Cancel; expTime += expTimeStep)
        {
            QCoreApplication::processEvents();

            // We change the camera exposure time
            newState.exposureUs = expTime;
            _camera->changeCameraParameters(EXPOSURE_TIME, newState);
            // We wait a little bit to leave the camera take a picture with the new exposure time
            QThread::msleep(100);

            // We take as many samples as the user wants, for the same exposure time
            for (int i = 0; i < samples && _camera->isAlive() && inProgressMsgBox.result() != QMessageBox::Cancel; i++)
            {
                QCoreApplication::processEvents();
                // We ensure the read image is not empty
                cv::Mat raw;
                while (raw.empty())
                {
                    _camera->getSingleImageSynchronously(raw);
                    QCoreApplication::processEvents();
                }

                cv::Mat correctedImg;
                bool useCalib = _useCalibrationCheckBox->checkState() == Qt::Checked;
                _pixelGainCalibratorHub.correctImage(raw, useCalib, correctedImg);
                _experimentsSaverLoader.addExperimentSample(correctedImg, expTime, true);
            }
            std::cout << "Samples taken for the exposure time = " << expTime << " uS" << std::endl;
        }
        /// We restore the camera state
        _camera->changeCameraParameters(ALL, oldState);
        onUpdateState(oldState);

        _experimentsSaverLoader.endExperiment();

        std::map<int, int> map = _filtersConfig.getPixelsMap();
        _pixelGainCalibratorHub.loadData(
            _loadSaveExpWidget->getSelectedKey(),
            {map[135], map[0], map[90], map[45]},
            _experimentsSaverLoader.getAngles(),
            _experimentsSaverLoader.getImages());

        _experimentsSaverLoader.reset();

        blockSignals(false);
        inProgressMsgBox.hide();
    }
    else
    {
        QMessageBox errorMessageBox;
        errorMessageBox.setIcon(QMessageBox::Warning);
        errorMessageBox.setStandardButtons(QMessageBox::Ok);
        QString description("ERROR! We cannot start the experiment. Check that\n");
        description += "there is not already an experiment with the same name,\n";
        description += "and that the working directory is writable.";
        errorMessageBox.setText(description);
        errorMessageBox.exec();
    }
    std::cout << "##################### Experiment finished! #####################" << std::endl;
}

void MainWindow::onCreateNewExperiment(int samples, int delta, std::string path, std::string expName)
{
    int idx = _pixelGainCalibratorHub.getCalibratorIndex(_loadSaveExpWidget->getSelectedKey());
    switch (idx)
    {
        case PIXEL_GAIN:
        {
            onCreateGainData(samples, delta, path, expName);
            break;
        }
        case BLACK_CURRENT:
        {
            onCreateBlackCurrentData(samples, delta, path, expName);
            break;
        }
        default:
        {
            assert(0);
            break;
        }
    }

    updateMatPlotLibSamplesLimit();
}

/// -------------- Plot related functions -------------- ///
void MainWindow::splitCalibratorImage(
    cv::Mat inputImg,
    const std::vector<double>& lightIntensities,
    const std::vector<double>& lightDoPs,
    std::vector<cv::Mat>& outputImages,
    std::vector<std::vector<double>>& outputLightParams,
    std::vector<std::string>& labels)
{
    std::vector<std::string> angleLabels({"135", "0", "90", "45"});
    std::vector<std::string> colorLabels({"Red", "Green_1", "Green_2", "Blue"});

    bool computeLightParams = false;
    std::vector<double> intensities;
    std::vector<double> dops;
    if (lightIntensities.size()  == 3 && lightDoPs.size() == 3)
    {
        // Since we have two green pixels per color pixel, we dupplicate the vector
        // value at the index 1. This way, we have a 1 to 1 correspondance between
        // the vectors colorLabels, intensities and dops.
        intensities = std::vector<double>({lightIntensities[0], lightIntensities[1], lightIntensities[1], lightIntensities[2]});
        dops = std::vector<double>({lightDoPs[0], lightDoPs[1], lightDoPs[1], lightDoPs[2]});
        computeLightParams = true;
    }
    else
    {
        std::cout << "The output light vector will not be filled since the input vector has not the right size" << std::endl;
    }
    outputLightParams.clear();

    /// The first split will divide the input image by polarization angle
    std::vector<cv::Mat> firstSplit;
    ProcessingParameters params = {
        .action = RAW_SPLITTED_IMAGES,
        .aop = -1,
        .filtersMap = _filtersConfig.getPixelsMap(),
        .imgFormatData = _camImgFormat
    };
    PolarimetricImagesProcessing::processImage(inputImg, params, firstSplit);

    /// The second split will divide each polarization angle by color. As a consequence,
    // we will obtain 16 images in outputImages
    outputImages.clear();
    labels.clear();

    for (unsigned int i = 0; i < firstSplit.size(); i++)
    {
        std::vector<cv::Mat> tempImgs;
        PolarimetricImagesProcessing::processImage(firstSplit[i], params, tempImgs);

        for (unsigned int j = 0; j < tempImgs.size(); j++)
        {
            outputImages.push_back(tempImgs[j].clone());
            labels.push_back(std::string("polarization angle: ") + angleLabels[i] + " and color: " + colorLabels[j]);
            if (computeLightParams)
            {
                outputLightParams.push_back({intensities[j], dops[j]});
            }
        }
    }
}

void MainWindow::onPlotDistribution(DistributionPlotType type)
{
    std::string calibrator = _loadSaveExpWidget->getSelectedKey();

    if (!_pixelGainCalibratorHub.areCalibrationComputed(calibrator))
    {
        std::cout << "We will not plot. The selected calibrator (" << calibrator << ") has not been computed yet" << std::endl;
        return;
    }

    cv::Mat imgToSplit;
    std::string plotTypeString;

    // These two parameters (S0 and DoP) are valid for the Gain calibration.
    // The black current calibrator does not need these params
    std::vector<double> lightS0;
    std::vector<double> lightDoP;

    if (calibrator == _pixelGainCalibratorHub.getCalibratorName(PIXEL_GAIN))
    {
        // We retrieve the source light information from the gain calibrator
        lightS0 = _pixelGainCalibratorHub.getCalibrationS0();
        lightDoP = _pixelGainCalibratorHub.getCalibrationDoP();
        switch (type)
        {
            case GAIN_PLOT:
            {
                cv::divide(_pixelGainCalibratorHub.getTiImage(), _pixelGainCalibratorHub.getPiImage(), imgToSplit);
                plotTypeString = "Gain";
                break;
            }
            case PHASE_PLOT:
            {
                imgToSplit = _pixelGainCalibratorHub.getPhaseShiftMatrix();
                plotTypeString = "Phase";
                break;
            }
            case TI_PLOT:
            {
                imgToSplit = _pixelGainCalibratorHub.getTiImage();
                plotTypeString = "Ti";
                break;
            }
            case PI_PLOT:
            {
                imgToSplit = _pixelGainCalibratorHub.getPiImage();
                plotTypeString = "Pi";
                break;
            }
            default:
            {
                std::cout << "ERROR: Requested plot type non recognized for the calibrator: " << calibrator << "!" << std::endl;
                return;
            }
        }
    }
    else if (calibrator == _pixelGainCalibratorHub.getCalibratorName(BLACK_CURRENT))
    {
        switch (type)
        {
            case BLACK_CURRENT_PLOT:
            {
                imgToSplit = _pixelGainCalibratorHub.getBlackCurrentMatrix();
                plotTypeString = "Black Current";
                break;
            }
            default:
            {
                std::cout << "ERROR: Requested plot type non recognized for the calibrator: " << calibrator << "!" << std::endl;
                return;
            }
        }
    }
    else
    {
        std::cout << "The selected calibrator is not being handled" << std::endl;
        return;
    }

    // These variables will be filled by the function splitCalibratorImage
    std::vector<cv::Mat> pixelImages;
    std::vector<std::string> pixelIdsStrings;
    std::vector<std::vector<double>> pixelLightParams;

    splitCalibratorImage(imgToSplit,
        lightS0,
        lightDoP,
        pixelImages,
        pixelLightParams,
        pixelIdsStrings);

    _matPlotLibWidget->createValueDistributionPlot(pixelImages, pixelIdsStrings, plotTypeString, _matPlotLibWidget->getParamDistMaxVal());
}

void MainWindow::onPlotSamples()
{
    std::string calibrator = _loadSaveExpWidget->getSelectedKey();
    if (_pixelGainCalibratorHub.isDataLoaded(calibrator))
    {
        _matPlotLibWidget->createSamplesPlot(
            _pixelGainCalibratorHub.getHorizontalAxis(calibrator),
            _pixelGainCalibratorHub.getRawSamples(calibrator));
    }
}

void MainWindow::onLiveDistPlot(LiveDistributionPlotType type)
{
    assert(_camera);
    cv::Mat raw;
    _camera->getSingleImageSynchronously(raw);

    if (!raw.empty())
    {
        std::vector<std::string> colorLabels = {"Red channel", "Green_1 channel", "Green_2 channel", "Blue channel"};
        std::string paramType;
        int startIdx = 0;

        bool useCalib = _useCalibrationCheckBox->checkState() == Qt::Checked;
        cv::Mat correctedImg;
        _pixelGainCalibratorHub.correctImage(raw, useCalib, correctedImg);
        std::vector<cv::Mat> imgs;
        ProcessingParameters params = {
            .action = RAW_I_RO_PHI,
            .aop = -1,
            .filtersMap = _filtersConfig.getPixelsMap(),
            .imgFormatData = _camImgFormat
        };
        PolarimetricImagesProcessing::processImage(correctedImg, params, imgs);

        // Sanity check
        assert(imgs.size() == 12);

        _matPlotLibWidget->closePlots();

        // All the images of the same type are grouped: The intensity images
        // are in the range 0 - 3, the angle of polarization images
        // are in the range 4 - 7, and the degree of polarization images
        // are in the range 8 - 11.
        switch(type)
        {
            case INTENSITY_PLOT:
            {
                startIdx = 0;
                paramType = "Intensity";
                break;
            }
            case AOP_PLOT:
            {
                startIdx = 4;
                paramType = "Angle of polarization";
                break;
            }
            case DOP_PLOT:
            {
                startIdx = 8;
                paramType = "Degree of polarization";
                break;
            }
            default:
            {
                std::cerr << "ERROR: Requested plot not found!!!!" << std::endl;
                assert(0);
                break;
            }
        }

        std::vector<cv::Mat> dataToPlot;
        dataToPlot.assign(imgs.begin() + startIdx, imgs.begin() + startIdx + 4);
        _matPlotLibWidget->createValueDistributionPlot(dataToPlot, colorLabels, paramType, _matPlotLibWidget->getLivePlotMaxVal());
    }
}

void MainWindow::onLiveRowPlotStateChanged(bool enabled)
{
    _isRunningRowPlotMode = enabled;
}

void MainWindow::executeHeavyTasks(threadFunc workerFnc, std::string msgTitle, std::string msgDescription)
{
    // Thread implementation inspired in https://stackoverflow.com/a/38770361
    QThread thread;
    QEventLoop myEventsLoop;
    QObject context;
    context.moveToThread(&thread);

    connect(&thread,
        &QThread::started,
        &context,
        [this, &myEventsLoop, workerFnc]() {
            (this->*workerFnc)();
            myEventsLoop.quit();
    });

    QMessageBox infoMsg;
    infoMsg.setIcon(QMessageBox::Information);
    infoMsg.setText(msgDescription.c_str());
    infoMsg.setWindowTitle(msgTitle.c_str());
    infoMsg.setStandardButtons(QMessageBox::NoButton);

    infoMsg.show();

    thread.start();
    myEventsLoop.exec();
    thread.quit();
    thread.wait();
    infoMsg.hide();
}
