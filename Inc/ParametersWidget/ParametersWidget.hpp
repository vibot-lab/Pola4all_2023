#ifndef __PARAMETERS_WIDGET_HPP__
#define __PARAMETERS_WIDGET_HPP__

// OpenCV includes

// STD includes
#include <vector>

// Qt Includes
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

// Custom includes
#include "CameraTypes.hpp"

/**
 * @brief ParametersWidget class
 *  This class is a QWidget that contitutes a control panel of all the parameters
 * we can change in the camera: Exposure time, frame rate, pixels gain,
 * specific color gains for white balance correction, and automatic features
 * (auto gain, auto exposure and auto white balance).
 *
 *  The gains can be regulated by sliding bars, and the frame rate and exposure
 * time as spinboxes.
 *
 *  Asynchoronous features has been implemented to interact with the outside:
 *      -) A signal is emitted each time a new state change has been produced.
 *      (any change in the gains, exposure time, for instance).
 *      -) A slot is available to receive the camera state. This one must be
 *      called in two specific moments:
 *          o) When the camera is initialized, so the reflected state is the
 *             actual state of the camera, and
 *          o) When the camera changes from an auto state (enabled or disabled).
 *          This is due to when we the auto state was enabled, the camera changed
 *          automatically the parameters involved with that feature. When we disable it,
 *          we want to know which is the value the camera set for those parameters.
 *
 *  When the slot function is executed, the signals triggering is disabled temporarly,
 * to avoid a back and forward signal emitting / receiving when the camera state
 * arrives.
*/
class ParametersWidget : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructor.
     *
     * @arg gainLimits: Vector with the limits for the sliders of the gain.
     *      Then the bar cannot be set with values lower or higher than those
     *      limits.
     *
     * @arg exposureLimits: Vector with the limits for the SpinBox of the exposure time.
     *      Then the values lower or higher than those limits cannot be entered.
     *
     * @arg parent: Parent for this QWidget. Default value set to zero.
    */
    ParametersWidget(
        std::vector<int> gainLimits,
        std::vector<double> exposureLimits,
        QWidget* parent = 0);
    ~ParametersWidget();

    /**
     * @brief setEnabled: Enable or disable this widget. This function will
     *      call the corresponding setEnabled functions of the different QWidgets
     *      that are part of this widget.
     *
     * @arg enabled: Boolean. If true, the widgets are enabled.
    */
    void setEnabled(bool enabled);

    /**
     * @brief setGainLimits: Change the gain limits in dB for the Slide Bars.
     *
     * @arg l: Vector with the gain limits. The first element of this vector
     *      represents the minimum value, and the second element represents
     *      the maximum value the slide bar can take. If more integers are
     *      present in this vector, they are ignored.
    */
    void setGainLimits(std::vector<int> l);

    /**
     * @brief setExposureLimits: Change the exposure time limits in uS for the spin box.
     *
     * @arg l: Vector with the exposure time limits. The first element of this vector
     *      represents the minimum value, and the second element represents
     *      the maximum value that can be entered in the spinbox. If more integers are
     *      present in this vector, they are ignored.
    */
    void setExposureLimits(std::vector<double> l);

    /**
     * @brief setSoftwareWhiteBalanceGains: Set the sliders values with the
     *  given gains. The internal variables are not set. This is function is used
     *  by the Software White balance module. We do not set the internal variables
     *  because that will imply to modify the API, and functions that are private
     *  would become public. While the sliders are modified, the corresponding
     *  signals are not emitted.
     *
     * @arg gains: Vector with the color gains to be set in the sliders. The slider
     *  value would be a float number with two one decimal accepted.
    */
    void setSoftwareWhiteBalanceGains(std::vector<double> gains);

    /**
     * @brief setAutoWhiteBalanceState: Set the state of the Auto white balance
     *  check box. If the check box state has been changed, the corresponding signal
     *  will not emitted inside this function. This function is used by the
     *
     * @arg enabled: If true, the auto white balance checkbox will be checked.
    */
    void setAutoWhiteBalanceState(bool enabled);

signals:
    /**
     * @brief updateParameters: Signal emitted with any parameter has been changed.
     *
     * @arg paramToChange: Element from the enum CameraParams that indicates
     *      which parameter we want to change.
     *
     * @arg state: A full state object. Only the parameter specified in paramToChange
     *      is valid inside this structure. All the other parameters are not initialized,
     *      so they must be ignored. Only if paramToChange = ALL, all the state
     *      is completely initialized
    */
    void updateParameters(CameraParams paramToChange, CameraState state);

public slots:
    /**
     * @brief onNewStateReceived: Slot that should be called each time the camera
     *      wants to show its state. Based on how the camera works, this slot
     *      must be called only when one of the auto features is disabled.
     *
     * @arg new_state: New full state of the camera. The struct must be
     *      completely initialized with the corresponding camera parameters.
     *      These values would be used to set the corresponding GUI widgets.
     *      When this operation is done, the signals of all the widgets in this
     *      module are disabled, in order to avoid spurious triggers.
    */
    void onNewStateReceived(const CameraState new_state);

protected slots:
    /**
     * @brief onGainChanged: Action to be executed when the gain slide bar
     *      change its value. It will emit the updateParameters signal, and
     *      update the corresponding label with the set value
     *
     * @arg value: Value set on the slide bar. Since this value is an integer,
     *      in order to have the decimal value to be emitted, this argument
     *      is divided by the _dpi variable, which is the amount of decimal positions
     *      we want to have.
    */
    void onGainChanged(int value);

    /**
     * @brief onExposureChanged: Slot executed when the exposure time spinbox change
     *      its value.
     *
     * @arg newVal: New value set through the spinbox. This exposure time,
     *      measured in uS, is send to the camera, using the signal updateparameters.
    */
    void onExposureChanged(double newVal);

    /**
     * @brief onAutoExposureChanged: Slot called whenever the auto exposure
     *      check box changes its state.
     *
     * @arg state. Integer with the new state of the checkbox. This change in the state
     *      is communicated to the outside emitting the signal updateparameters.
    */
    void onAutoExposureChanged(int state);

    /**
     * @brief onAutoGainChanged: Slot called whenever the auto gain
     *      check box changes its state.
     *
     * @arg state. Integer with the new state of the checkbox. This change in the state
     *      is communicated to the outside emitting the signal updateparameters.
    */
    void onAutoGainChanged(int state);

    /**
     * @brief onFramerateChanged: Slot called whenever the user changes the frame
     *      rate through the corresponding spinbox. The new value will be
     *      communicated to the camera through the signal updateparameters.
     *
     * @arg value: New frame rate value set by the user.
    */
    void onFramerateChanged(int value);

    /**
     * @brief onAutoWhiteBalance: Slot called whenever the auto white balance
     *      check box changes its state.
     *
     * @arg state. Integer with the new state of the checkbox. This change in the state
     *      is communicated to the outside emitting the signal updateparameters.
    */
    void onAutoWhiteBalance(int state);

    /**
     * @brief onWhiteBalanceChanged: Slot called whenever any of the white
     *      balance gain slide bar is changed. When this function is called
     *      the three gains are sent to the outside through the signal
     *      updateparameters.
     *
     * @arg value: New value of the slide bar that has changed. This value is
     *      ignored, and the three gains are read in this function.
    */
    void onWhiteBalanceChanged(int value);

private:
    /**
     * @brief initializeWidget: Initialize class widgets and layout.
    */
    void initializeWidget();

    /**
     * @brief addGainControlBar: Function used during initialization. It adds
     *      the gain slide bar to the widget layout.
    */
    void addGainControlBar();

    /**
     * @brief addExposureControlSpinbox: Function used during initialization. It adds
     *      the exposure time spin box to the widget layout.
    */
    void addExposureControlSpinbox();

    /**
     * @brief addFramerateSpinbox: Function used during initialization. It adds
     *      the frame rate spin box to the widget layout.
    */
    void addFramerateSpinbox();

    /**
     * @brief addWhiteBalanceBars: Function used during initialization. It adds
     *      the three white balance slide bars to the widget layout.
    */
    void addWhiteBalanceBars();

    /**
     * @brief createSlideBar: Add a slide bar to the provided layout, and
     *      it initializes the given QLabels to show the actual value of the bar.
     *
     * @arg title: Name of the label that will be placed at the left of the
     *      slide bar, that indicates what it is.
     *
     * @arg initialTick: Initial value of the label that indicates the actual
     *      tick of the slide bar.
     *
     * @arg slidePtr: Reference to the pointer where we will store the slide
     *      bar object. We do not check the pointer previous value. If the object
     *      risks to be previously initialized, the deletion must be handled
     *      outside this function.
     *
     * @arg labelTickPtr: Reference to the pointer where the tick's label will be
     *      stored. This label is the one that will show the actual value of the
     *      parameter handled by the created slide bar.
     *
     * @arg outputLayout: In this layout, the slide bars will be placed. It must
     *      be initialized outside the function. Inside, this pointer is
     *      used directly.
    */
    void createSlideBar(std::string title,
        std::string initialTick,
        QSlider* &sliderPtr,
        QLabel* &labelTicksPtr,
        QFormLayout* &outputLayout);

    QVBoxLayout* _mainLayout;
    QLabel* _gainTicks;
    QLabel* _redGainTicks;
    QLabel* _greenGainTicks;
    QLabel* _blueGainTicks;

    QSlider* _gainControlWidget;
    QDoubleSpinBox* _exposureControlWidget;
    QSpinBox* _framerateControlWidget;

    QSlider* _redBalance;
    QSlider* _greenBalance;
    QSlider* _blueBalance;

    QCheckBox* _autoGainCheckbox;
    QCheckBox* _autoExposureCheckbox;
    QCheckBox* _autoWhiteBalanceCheckbox;

    std::vector<int> _gainLimits;
    std::vector<double> _exposureLimits;
    int _dpi;
    double _lastExposureVal;
    int _lastExposureState;
    int _lastGainState;
    int _lastFrameRate;
};

#endif // __PARAMETERS_WIDGET_HPP__
