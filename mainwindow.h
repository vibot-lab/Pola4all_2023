#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// OpenCV includes
#include <opencv2/core/core.hpp>

// STD includes
#include <map>

// Qt Includes
#include <QComboBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QString>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

// Custom includes
#include "CameraStateCallback.hpp"
#include "CameraTypes.hpp"
#include "FilterOrientationsWidget/FilterOrientationsWidget.hpp"
#include "ImageWidget/ImageGridWidget.hpp"
#include "LoadSaveWidget/LoadSaveWidget.hpp"
#include "MatPlotLibPlotsWidget/MatPlotLibPlotsWidget.hpp"
#include "ParametersWidget/ParametersWidget.hpp"
#include "PolarimetricCamera/LoadSaveExperiments/LoadSaveExperiments.hpp"
#include "PolarimetricCamera/PixelGainCalibration/CalibratorsHub.hpp"
#include "PolarimetricCamera/PixelGainCalibration/SourceOrientationEstimator.hpp"
#include "PolarimetricCamera/RosCamClient/RosCamClient.hpp"
#include "RealTimeParameters/RealTimeParameters.hpp"
#include "SaveImagesWidget/SaveImagesWidget.hpp"
#include "SoftwareWhiteBalance/SoftwareWhiteBalance.hpp"
#include "VisualizationModeWidget/VisualizationModeWidget.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @brief MainWindow class
 *
 *  This is the first and only class that manages all the GUI interface.
 * It holds all the widgets, modules interactions, and oganizes the flow of the
 * application.
 *
 *  It inherits also from CameraStateCallback so we can pass it as a pointer
 * to this class to other class, and we avoid the Qt dependency on the latter ones.
 *
 *  This is the biggest class in the program, since all the modules are independent
 * one each other, so that's why all the interactions are done here.
*/

/// Typedef required to define the callback functions when we execute the threads.
class MainWindow;
typedef void (MainWindow::*threadFunc)();

class MainWindow : public QMainWindow, public CameraStateCallback
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     *
     * @arg camImgFormat: Object with the camera image bit-depth information. It
     *   is a shared pointer, so that we do not copy it from one function to another,
     *   and only a single copy of it is created in the entire program.
    */
    MainWindow(std::shared_ptr<bitDepthStruct> camImgFormat, QWidget *parent = nullptr);
    ~MainWindow();

protected slots:
    /**
     * @brief onConnectCamera: Slot called whenever the user pushes the Connect
     *      to camera button. It will show the dialog that request the user to
     *      check which connection type he wants, and then it will try to connect
     *      to the camera in that way. If it does not fail, it will try to put
     *      the camera into free-run mode, it will update the corresponding
     *      labels, and it will start the corresponding timers.
     *       If it fails, it will reset the GUI as if the camera was disconnected.
    */
    void onConnectCamera();

    /**
     * @brief onDisconnectCamera: Disconnect the camera, and reset the GUI to the
     *      disconnect state. If the camera has been initialized, it will put it
     *      into idle mode, clear the showed image, and reset the GUI state.
    */
    void onDisconnectCamera();

    /**
     * @brief onCaptureImageRequest: Slot called whenever the user wants to
     *      save an image. It will temporarily stop the grabbing timer, it will
     *      capture a single image. After, it will call the SaveImagesWidget::onImagesReceived
     *      slot, and finally it will restart the grabbing timer.
    */
    void onCaptureImageRequest();

    /**
     * @brief onImageWidthChanged: Slot called each time the ImageWidth value
     *  changes.
     *
     * @arg: Value entered by the user to change the ImageWidget width, measured
     *  in pixels
    */
    void onImageWidthChanged(int value);

    /**
     * @brief updateCurrentLabels: Update the labels stored in the modules that
     *      make use of the them with the last ones corresponding to the user
     *      selected processing level.
    */
    void updateCurrentLabels();

    /**
     * @brief updateMatPlotLibSamplesLimit: Update the SpinBox limits, based on
     *  the image size. This image are the ones stored in the calibrator hub. This
     *  function must be called each time a new experiment is created, or we switch
     *  the calibrator module. These limits are stored in the MatPlotLibPlots module.
    */
    void updateMatPlotLibSamplesLimit();

    /**
     * @brief onExit: Function called whenever the user presses the button Exit,
     *      or it closes the program. If the camera has been connected at least once,
     *      the program will disconnect from it, leaving the control to anybody
     *      who wants to connect to it.
    */
    void onExit();

    /**
     * @brief updateCameraParameters: Slot called each time a camera parameter
     *      has been changed from the ParametersWidget class, and it needs to be
     *      tramitted to the camera.
     *
     * @arg paramToChange: Enum value that informs which parameter has been changed.
     * @arg state: Struct with all the possible camera parameters we can change.
     *       If paramToChange is ALL, all the fields of this struct have meaningful
     *      values. In any other case, only the field pointed by paramToChange
     *      has been set, and the other fields are uninitialized, and they must
     *      be ignored.
    */
    void updateCameraParameters(CameraParams paramToChange, CameraState state);

    /**
     * @brief onUpdateState: Slot called each time the camera thread wants
     *  to inform us of a new state. It is useful to switch from the second
     *  thread to the main thread. If we do not do this, a segmentation fault
     *  will occur, since (by definition), a QWidget CANNOT be updated from any
     *  thread that is not the main one. Using a signal will allow us that the
     *  update will happen in the main thread.
     *
     * @arg newState: State received from the camera.
    */
    void onUpdateState(CameraState newState);

    /**
     * @brief onCreateGainData: Slot called each time the user wants to
     * create a new experiment for the pixel gain calibrator.
     * This experiment consists of taking several images with the polarization
     * camera. The camera must be placed in front of a linearly polarized
     * light, and take several samples at different polarization
     * angles. Per angle, we take several images, and we do the average of them.
     *  For this experiment, we store the grabbed images, so we do not have to
     * make it again.
     *
     * @arg samples: Amount of samples to take for each polarization angle.
     * @arg angleStep: Step to increase the polarization angle, after taking
     *  "samples" samples.
     * @arg path: Current working directory
     * @arg expName: Experiment name. The grabbed images will be stored in the
     * working directory, in a directory called as the value stored in this variable.
    */
    void onCreateGainData(int samples, int angleStep, std::string path, std::string expName);

    /**
     * @brief onCreateBlackCurrentData: Slot called each time the user wants to
     * create a new experiment for the pixel offset calibrator.
     * This experiment consists of taking several images with the polarization
     * camera. The camera cover must be placed in the lens, in order to block light
     * that arrives to the camera. Then, this function will take several pictures,
     * while changing the exposure time. For each exposure time, we take several
     * images, and we average them, in order to reduce the influence of the noise.
     *  For this experiment, we store the grabbed images, so we do not have to
     * make it again.
     *
     * @arg samples: Amount of samples to take for each exposure time.
     * @arg expTimeStep: Step to increase the exposure time, after taking
     *  "samples" samples.
     * @arg path: Current working directory
     * @arg expName: Experiment name. The grabbed images will be stored in the
     * working directory, in a directory called as the value stored in this variable.
    */
    void onCreateBlackCurrentData(int samples, int expTimeStep, std::string path, std::string expName);

    /**
     * @brief onCreateNewExperiment: Slot called each time the user requests to
     *  create a new experiment. This function will be in charge of deciding if
     *  to call either, onCreateGainData or onCreateBlackCurrentData.
     *
     * @arg samples: Amount of samples to take for each exposure time.
     * @arg delta: Step to increase the horizontal variable value, after taking
     *  "samples" samples.
     * @arg path: Current working directory
     * @arg expName: Experiment name. The grabbed images will be stored in the
     * working directory, in a directory called as the value stored in this variable.
    */
    void onCreateNewExperiment(int samples, int delta, std::string path, std::string expName);

    /**
     * @brief onLoadExperimentRequest: Slot executed each time the user wants to load
     * the data from a previous experiment. The loaded data is the corresponding
     * data stored by the function onCreateExperimentData.
     *  This function will trigger an independent thread, since the data loading
     * operation can take a long time, and hung up the GUI.
     *
     * @arg path: Current where the experiment is located.
     * @arg expName: Experiment name. This folder will be located at "path",
     * in a directory that holds the name stored in this variable.
    */
    void onLoadExperimentRequest(std::string expFolder, std::string expName);

    /**
     * @brief onPlotAmpDistribution: Slot called each time the MatPlotLibPlotsWidget
     *  asks to plot the calibration distribution for the amplitude. This function
     *  will take the data from PixelGainCalibrator module, and it will send it
     *  to the MatPlotLibPlotsWidget module. This way, we avoid the inter-module
     *  dependence.
     *
     * @arg type: Enum value that tells which data we take from the calibrator
     * to plot.
    */
    void onPlotDistribution(DistributionPlotType type);

    /**
     * @brief onPlotSamples: Slot called each time the MatPlotLibPlotsWidget
     *  asks to plot a set of samples. This function will take the data from
     *  the corresponding modules, and it will send it to the MatPlotLibPlotsWidget
     *  module. This way, we avoid the inter-module dependence.
    */
    void onPlotSamples();

    /**
     * @brief splitCalibratorImage: Split the calibration image into
     *  different images. Each split image contains all the calibration values
     *  for one of the pixels from the super-pixel array.
     *
     *      In order to split the images, two split are done: The first one
     *  divides the image by polarization angle, and for each of these images,
     *  we split them again by color. For the Basler acA2440-75ucPOL, this implies
     *  we will have 16 images.
     *
     * @arg inputImg: Single parameter image.
     *
     * @arg lightIntensities: Vector with the input light intensities.
     *      Each element corresponds to the intensity of each color component (red,
     *      green and blue, respectively).
     *
     * @arg lightDoPs: Vector with the input light degree of polarization.
     *      Each element corresponds to the intensity of each color component (red,
     *      green and blue, respectively).
     *
     * @arg outputImages [output]: Images extracted from the input data.
     *
     * @arg outputLightParams [output]: Vector of 2D vectors. Each element contains the intensity
     *      value and the degree of polarization that corresponds to light detected
     *      by the pixel we have at each position
     *
     * @arg labels [output]: Array with strings that identifies which pixel we have at each position.
    */
    void splitCalibratorImage(
        cv::Mat inputImg,
        const std::vector<double>& lightIntensities,
        const std::vector<double>& lightDoPs,
        std::vector<cv::Mat>& outputImages,
        std::vector<std::vector<double>>& outputLightParams,
        std::vector<std::string>& labels);

    /**
     * @brief onCalibratePixels: Slot called each time the button "Compute pixel gain calibration"
     * is clicked. It will compute the gain and phase shift matrices for each pixel,
     * and it will compute the calibration matrix for the pixel gain.
     *      This slot must be called after the experiment data has been created
     * or loaded from local files.
     *  This function will trigger an independent thread, since the calibration
     * algorithm can take a long time, and hung up the GUI.
    */
    void onCalibratePixels();

    /**
     * @brief onLiveDistPlot: Slot executed each time the user wants to create a
     *  a distribution plot of the current image being shown.
     *
     * @arg type: The type of plot we want to make: intensity, DoLP or AoLP.
    */
    void onLiveDistPlot(LiveDistributionPlotType type);

    /**
     * @brief onTestDifferentROI: Slot executed each time the button
     *  AoLP estimator ROI test is pressed. It will start a new thread
     *  to excute the task, without hanging the GUI.
    */
    void onTestDifferentROI();

    /**
     * @brief runROIChangeErrorInAoLP: Thread function executed in onTestDifferentROI.
     *   In this test, we compute the maximum error and the RMSE of the AoLP
     * estimator for different values of the central region. The region size increase
     * step is 2 to take the correct pixels for each super-pixel. The central region is
     * considered to be squared, and the maximum size is the sensor's minimum size.
     *
     *   Additionally, an experiment should be loaded previously to execute this thread.
     *  Once finished, the test will print three vectors in the standard output:
     * The different ROI sizes considered, the maximum error found for each of these regions,
     * and the RMSE for each region size. The vectors are printed such that they can be
     * imported in a Python script to be further analyzed. To have correct results, the
     * ground truth angles should be the ones specified in the filenames of the loaded
     * images.
    */
    void runROIChangeErrorInAoLP();

    /**
     * @brief computeAoLPTestMetrics: Compute the evaluation metrics for the AoLP estimator
     *  test, given the estimated angles, and the ground truth AoLP. For now, only the
     *  maximum value and the RMSE value are computed as metrics.
     *
     * @arg estAngles: Vector with the estimated AoLP.
     * @arg gtAngles: Vector with the ground truth AoLP. This vector must have the same size
     *  as estAngles, and the corresponding elements should correspond to the same sample.
     * @arg maxVal [output]: Maximum error found for all the given samples.
     * @arg rmseVal [output]: RMSE found for all the given samples.
    */
    void computeAoLPTestMetrics(const std::vector<double> &estAngles, const std::vector<double>& gtAngles, double &maxVal, double &rmseVal);

    /**
     * @brief onUpdateFilterOrientations: Slot executed each time the user changes
     *   the camera filter orientations. This function will update the modules that
     *   depends on this information.
     *
     * @arg map: Map from the filter orientations (in degrees), and their positions
     *   in the 2x2 array of the super-pixel.
    */
   void onUpdateFilterOrientations(std::map<int,int> map);

signals:
    /**
     * @brief updateState: Signal emitted each time the camera thread wants
     *  to inform us of a new state. It is useful to switch from the second
     *  thread to the main thread. If we do not do this, a segmentation fault
     *  will occur, since (by definition), a QWidget CANNOT be updated from any
     *  thread that is not the main one. Using a signal will allow us that the
     *  update will happen in the main thread.
     *
     * @arg newState: State sent by the camera.
    */
    void updateState(CameraState newState);

private:
    /**
     * @brief initializeWidgets: Main initialization function. This method will
     *      initialze the corresponding widgets, and at the end, it will set
     *      the custom layout to be the MainWindow's layout. Inside
     *      this function, THE ORDER MATTERS. This means that if we switch the
     *      order of the widgets initialization, it will change the position
     *      of them in the GUI. Furthermore, some widgets require others to be
     *      initialized before. If we wrongly change their order, segmentation
     *      fault can occur, or some connections might not be done properly.
    */
    void initializeWidgets();

    /**
     * @brief initializeMainWidowLayouts: Function that establishes the MainWindow
     *      layout. The layout is organized as two parts: Left and right.
     *       The left layout handles the images, and the right one handles the
     *      camera parameters, and buttons.
    */
    void initializeMainWidowLayouts();

    /// \brief addImageWidget: Add the ImageGridWidget to the MainWindow
    void addImageWidget();

    /// \brief addButtons: Add the connect, disconnect and test buttons to the MainWindow, and its connections
    void addButtons();

    /// \brief addParametersWidget: Add the ParametersWidget to the MainWindow, and its connections
    void addParametersWidget(QLayout* layout, QWidget* parent);

    /// \brief addSaveImagesWidget: Add the SaveImageWidget to the MainWindow, and its connections
    void addSaveImagesWidget(QLayout* layout, QWidget* parent);

    /// \brief addExitButton: Add the exit button to the MainWindow, and its connections
    void addExitButton();

    /// \brief initializeTemperatureLabel: Add the temperature label to the MainWindow, and its connections
    void initializeTemperatureLabel(QLayout* layout, QWidget* parent);

    /// \brief addImageWidthSpinBox Add an spinbox and a label to change the ImageWidget width
    void addImageWidthSpinBox(QLayout* layout, QWidget* parent);

    /// \brief addPlotsWidget: Add the MatPlotLibPlotsWidget to the mainwindow.
    void addPlotsWidget(QLayout* layout, QWidget* parent);

    /// \brief addCalibrationCheckBox: Add a checkbox to correct the image based on the pixel gain calibration matrix
    void addCalibrationCheckBox();

    /// \brief addModeWidget: Add the widget that allows to change the processing level.
    void addModeWidget(QLayout* layout, QWidget* parent);

    /// \brief addAoLPEstimatorTestButton: Add a button to execute the AoLP estimator ROI error test 
    void addAoLPEstimatorTestButton(QLayout* layout, QWidget* parent);

    /**
     * @brief createTabOne: Add the first tab widget, that corresponds to the camera control.
     *  This function creates a single widget with all the widgets that comprise the
     *  camera control module: Changing camera parameters, changing the camera mode,
     *  changing the gain, exposure time, and the white balance.
     *
     * @returns: QWidget whose layout includes all the camera control widgets
    */
    QWidget* createTabOne();

    /**
     * @brief createTabTwo: Add the second tab widget, that corresponds to the camera calibration.
     *  This function creates a single widget with all the widgets that comprise the
     *  camera calibration module. The module allows to create or to load existing
     *  data for the calibration, and to compute the calibration parameters. The
     *  module allows also to decide if we want to calibrate either, the black
     *  current or the pixel correction part of the flat-field calibration.
     *
     * @returns: QWidget whose layout includes all the camera calibration widgets
    */
    QWidget* createTabTwo();

    /**
     * @brief createTabThree: Add a third tab with all the plotting functionalities.
     *   This tab includes the functionalities to plot the input samples used for
     *   the calibration, and the distribution plots of the found parameters.
     *
     * @returns: QWidget whose layout includes all the plotting widgets
    */
    QWidget* createTabThree();

    /**
     * @brief addPixelCalibrationWidget: Function that adds all the widgets required
     * to make the pixel gains calibration.
     *
     * @arg layout: Layout to add the widgets to.
     * @arg parent: Parent widget
    */
    void addPixelCalibrationWidget(QLayout* layout, QWidget* parent);

    /**
     * @brief addParametersShowingWidget: It adds to the layout a group box
     * of widgets composed of: A checkbox to enable or disable the functionality,
     * and two labels. One label to indicate the maximum intensity value measured,
     * and a second label to show the average angle of polarization. If
     * enabled, both labels are updated each time a new image arrives.
     *
     * @arg localLayout: Pointer to the layout where we want to place this widget.
     * @arg parent: Parent widget
    */
    void addParametersShowingWidget(QLayout* localLayout, QWidget* parent);

    /**
     * @brief resetGuiState: Change all the widgets related to the camera
     *      connection to its default state. This will be done, for instance,
     *      when the application disconnects from the camera.
    */
    void resetGuiState();

    /**
     * @brief setButtonsEnabled: Enable / disable all the parameters widgets in the
     *      MainWindow GUI. The Connect to camera button follows the complementary
     *      state of the other buttons, i.e., the connect to camera button is
     *      enabled when all the other widgets are disabled, and vice versa. The
     *      exit button is always enabled.
     *
     * @arg enabled: Boolean. If true, all the widgets will be enabled, and the
     *      connect to camera button will be disabled.
    */
    void setButtonsEnabled(bool enabled);

    // Intermediate callbacks to talk with the camera
    /**
     * @brief updateStateCallback: Slot received from the camera each time the camera
     *      wants to inform a change in its state. Generally, this happens
     *      during initialization, and when a auto State has been changed.
     *
     * @arg newState: Struct with all the camera parameters initialized with
     *      the actual camera values.
    */
    void updateStateCallback(const CameraState &newState) override;

    /**
     * @brief refreshCameraData: Update all the camera data that is being showed
     *  in the GUI (image, temperature, ...). This is a QTimer timeout callback.
    */
    void refreshCameraData();

    /**
     * @brief refreshRealTimeParams: Update the real time parameters computed from the
     *  received image. This is done only if the corresponding checkbox is enabled.
     *  This is a QTimer timeout callback. Since this function
     *  takes a lot of time, its frequency is slower than for the refresh
     *  function that shows the received image.
    */
    void refreshRealTimeParams();

    /**
     * @brief executeHeavyTasks: Place a function in a separated thread, to
     * void hanging the GUI. The worker function should be a member of the
     * MainWindow class. When the thread is started, a message box will be shown,
     * saying that the program is doing a computation, and it cannot continue.
     * This box will block the rest of the main window, but we can move it and
     * resize. When the thread finishes, the pop-up will disappear.
     *  A QEventLoop object is used to refresh the GUI, and void hanging it.
     *
     * @arg workerFunc: Callback to be executed in the secondary thread.
     * @arg msgTitle: Title to place in the Message box to be shown.
     * @arg msgDescription: Description message to place in the Pop-up object.
    */
    void executeHeavyTasks(threadFunc workerFnc, std::string msgTitle, std::string msgDescription);

    /**
     * @brief onLiveRowPlotStateChanged: Change the flag state that enableds / disables
     *   the live row plot behavior.
     *
     * @param enabled: If true, the live row plot feature is enabled. In that case,
     *   the image coming for the camera is not updated, but only the row plots.
     */
    void onLiveRowPlotStateChanged(bool enabled);

    /**
     * @brief loadExperimentData: Thread function. This slot is called when the
     * loading thread is executed. It will load the data as a background process.
    */
    void loadExperimentData();

    /**
     * @brief runPixelsCalibrations: Thread function. This slot is called when the
     * calibration thread is executed. It will compute the calibration matrices
     * as a background process.
    */
    void runPixelsCalibrations();

    /**
     * @brief createStringWithNumber: Create an string that is a combination of
     * a number and a string.
     *
     * @arg txt: Text to be place at the beginning of the string. No spaces are
     *  added between the number and the string, so this string must include
     *  a trailing space.
     * @arg number: Decimal number to be placed after the string.
     * @arg precision: Amount of decimals to include in the number.
     * @returns String with the join of the given text and the given number.
    */
    std::string createStringWithNumber(std::string txt, double number, int precision);

    /// Layouts
    QVBoxLayout *_leftLayout;
    QVBoxLayout *_rightLayout;
    QHBoxLayout *_mainLayout;

    /// Custom widgets
    ImageGridWidget *_imageWidget;
    LoadSaveWidget *_loadSaveExpWidget;
    MatPlotLibPlotsWidget* _matPlotLibWidget;
    ParametersWidget *_paramsWidget;
    SaveImagesWidget *_saveImgWidget;

    /// Labels
    QLabel *_messageLabel;
    QLabel *_tempLabel;

    /// Scroll area for the Image Widget
    QScrollArea *_scroll;

    /// Buttons
    QPushButton *_connect;
    QPushButton *_disconnect;
    QPushButton *_exit;

    /// Timers timeouts
    int _refreshCameraPeriod;
    int _refreshRTParamsPeriod;

    /// Timers
    QTimer _refreshDataTimer;
    QTimer _refreshRealTimeParamsTimer;

    // QCheckBox
    QCheckBox* _useCalibrationCheckBox;
    QCheckBox* _useSuperPixelCheckBox;

    // QSpinBox
    QSpinBox* _imageWidth;

    // QTabWidget
    QTabWidget* _tabs;

    /// Internal Camera object
    RosCamClient* _camera;

    /// Qt compiled GUI. It is empty. All the GUI is designed by code.
    Ui::MainWindow *ui;

    /// Internal variables
    FilterOrientationsWidget _filtersConfig;
    SoftwareWhiteBalance _whiteBalanceControl;
    LoadSaveExperiments _experimentsSaverLoader;
    CalibratorsHub _pixelGainCalibratorHub;

    // These two variables by the data loading thread
    std::string _expFolder;
    std::string _expName;

    // This module serves to estimate the average AoLP of a given image.
    SourceOrientationEstimator _avgAngleEstimation;
    RealTimeParameters* _liveParamsWidget;
    VisualizationModeWidget* _modeWidget;

    bool _isRunningRowPlotMode;
    std::shared_ptr<bitDepthStruct> _camImgFormat;
};

#endif // MAINWINDOW_H