// OpenCV includes

// STD includes
#include <iostream>
#include <sstream>

// Qt Includes
#include <QGroupBox>

// Custom includes
#include "ParametersWidget/ParametersWidget.hpp"

ParametersWidget::ParametersWidget(
    std::vector<int> gainLimits,
    std::vector<double> exposureLimits,
    QWidget* parent) :
    QWidget(parent),
    _mainLayout(nullptr),
    _gainTicks(nullptr),
    _redGainTicks(nullptr),
    _greenGainTicks(nullptr),
    _blueGainTicks(nullptr),
    _gainControlWidget(nullptr),
    _exposureControlWidget(nullptr),
    _framerateControlWidget(nullptr),
    _redBalance(nullptr),
    _greenBalance(nullptr),
    _blueBalance(nullptr),
    _autoGainCheckbox(nullptr),
    _autoExposureCheckbox(nullptr),
    _autoWhiteBalanceCheckbox(nullptr),
    _gainLimits(gainLimits),
    _exposureLimits(exposureLimits),
    _dpi(10),
    _lastExposureVal(0),
    _lastExposureState(0),
    _lastGainState(0),
    _lastFrameRate(0)
{
    // IMPORTANT: If you add more types to the CameraParams class, be sure
    // to add the function in this class
    assert(NUM_OF_PARAMS == 8);
    initializeWidget();
}

ParametersWidget::~ParametersWidget()
{
    delete _mainLayout;
    delete _gainTicks;
    delete _redGainTicks;
    delete _greenGainTicks;
    delete _blueGainTicks;
    delete _gainControlWidget;
    delete _exposureControlWidget;
    delete _framerateControlWidget;
    delete _redBalance;
    delete _greenBalance;
    delete _blueBalance;
    delete _autoGainCheckbox;
    delete _autoExposureCheckbox;
    delete _autoWhiteBalanceCheckbox;
}

void ParametersWidget::initializeWidget()
{
    _mainLayout = new QVBoxLayout(this);
    addGainControlBar();
    addExposureControlSpinbox();
    addWhiteBalanceBars();
    addFramerateSpinbox();

    setEnabled(false);
}

void ParametersWidget::setEnabled(bool enabled)
{
    _exposureControlWidget->setEnabled(enabled);
    _gainControlWidget->setEnabled(enabled);
    _autoGainCheckbox->setEnabled(enabled);
    _autoExposureCheckbox->setEnabled(enabled);
    _framerateControlWidget->setEnabled(enabled);
    _autoWhiteBalanceCheckbox->setEnabled(enabled);
    _redBalance->setEnabled(enabled);
    _greenBalance->setEnabled(enabled);
    _blueBalance->setEnabled(enabled);
}

void ParametersWidget::addGainControlBar()
{
    QGroupBox* formWidget = new QGroupBox("Gain control", this);
    QWidget* sliderWidget = new QWidget(formWidget);
    QFormLayout* labelAndSliderLayout = new QFormLayout(formWidget);
    QVBoxLayout* sliderTicksLayout = new QVBoxLayout(sliderWidget);

    // Create a QWidget with a slider and its ticks as label
    _gainControlWidget = new QSlider(Qt::Horizontal, sliderWidget);
    _gainTicks = new QLabel("0 dB", sliderWidget);
    sliderTicksLayout->addWidget(_gainControlWidget);
    sliderTicksLayout->addWidget(_gainTicks);
    sliderWidget->setLayout(sliderTicksLayout);

    // We create a label to identify the slider
    QLabel* textLabel = new QLabel("Gain control", formWidget);
    // We add the slider and its label into a form layout
    labelAndSliderLayout->addRow(textLabel, sliderWidget);

    _autoGainCheckbox = new QCheckBox("Auto gain", formWidget);
    labelAndSliderLayout->addRow(_autoGainCheckbox);

    // We add everything to the main layout
    formWidget->setLayout(labelAndSliderLayout);
    _mainLayout->addWidget(formWidget);

    _gainControlWidget->setRange(_gainLimits[0] * _dpi, _gainLimits[1] * _dpi);

    connect(_autoGainCheckbox,
        &QCheckBox::stateChanged,
        this,
        &ParametersWidget::onAutoGainChanged);

    connect(_gainControlWidget,
        &QSlider::valueChanged,
        this,
        &ParametersWidget::onGainChanged);
}

void ParametersWidget::addExposureControlSpinbox()
{
    QGroupBox* formWidget = new QGroupBox("Exposure control", this);
    QFormLayout* labelAndSpinboxLayout = new QFormLayout(formWidget);

    // Create a QWidget with a slider and its ticks as label
    _exposureControlWidget = new QDoubleSpinBox(formWidget);

    // We create a label to identify the slider
    QLabel* textLabel = new QLabel("Exposure time", formWidget);
    // We add the slider and its label into a form layout
    labelAndSpinboxLayout->addRow(textLabel, _exposureControlWidget);

    // We add the AutoControl checkbox at the bottom
    _autoExposureCheckbox = new QCheckBox("Auto exposure", formWidget);
    labelAndSpinboxLayout->addRow(_autoExposureCheckbox);

    // We add everything to the main layout
    formWidget->setLayout(labelAndSpinboxLayout);
    _mainLayout->addWidget(formWidget);

    _exposureControlWidget->setMinimum(_exposureLimits[0]);
    _exposureControlWidget->setMaximum(_exposureLimits[1]);
    _exposureControlWidget->setDecimals(1);
    _exposureControlWidget->setSingleStep(0.1);
    _exposureControlWidget->setSuffix(" uS");

    connect(_exposureControlWidget,
        QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this,
        &ParametersWidget::onExposureChanged);

    connect(_autoExposureCheckbox,
        &QCheckBox::stateChanged,
        this,
        &ParametersWidget::onAutoExposureChanged);
}

void ParametersWidget::addFramerateSpinbox()
{
    QWidget* formWidget = new QWidget(this);
    QFormLayout* labelAndSpinboxLayout = new QFormLayout(formWidget);

    // Create a QWidget with a slider and its ticks as label
    _framerateControlWidget = new QSpinBox(formWidget);

    // We create a label to identify the slider
    QLabel* textLabel = new QLabel("Frame rate", formWidget);
    // We add the slider and its label into a form layout
    labelAndSpinboxLayout->addRow(textLabel, _framerateControlWidget);

    // We add everything to the main layout
    formWidget->setLayout(labelAndSpinboxLayout);
    _mainLayout->addWidget(formWidget);

    _framerateControlWidget->setMinimum(1);
    _framerateControlWidget->setMaximum(70);
    _framerateControlWidget->setSuffix(" fps");

    connect(_framerateControlWidget,
        QOverload<int>::of(&QSpinBox::valueChanged),
        this,
        &ParametersWidget::onFramerateChanged);
}

void ParametersWidget::createSlideBar(std::string title,
    std::string initialTick,
    QSlider* &sliderPtr,
    QLabel* &labelTicksPtr,
    QFormLayout* &outputLayout)
{
    delete sliderPtr;
    delete labelTicksPtr;

    QWidget* widget = new QWidget(this);
    QLabel* titleLabel = new QLabel(title.c_str(), this);
    QVBoxLayout* layout = new QVBoxLayout(widget);

    // Create a QWidget with a slider and its ticks as label
    sliderPtr = new QSlider(Qt::Horizontal, widget);
    labelTicksPtr = new QLabel(initialTick.c_str(), widget);
    layout->addWidget(sliderPtr);
    layout->addWidget(labelTicksPtr);
    // We create a label to identify the slider
    widget->setLayout(layout);

    outputLayout->addRow(titleLabel, widget);
}

void ParametersWidget::addWhiteBalanceBars()
{
    QGroupBox* formWidget = new QGroupBox("White Balance", this);
    QFormLayout* labelAndSliderLayout = new QFormLayout(formWidget);

    createSlideBar("Red gain control", "1.0", _redBalance, _redGainTicks, labelAndSliderLayout);
    createSlideBar("Green gain control", "1.0", _greenBalance, _greenGainTicks, labelAndSliderLayout);
    createSlideBar("Blue gain control", "1.0", _blueBalance, _blueGainTicks, labelAndSliderLayout);

    _autoWhiteBalanceCheckbox = new QCheckBox("Auto white balance", formWidget);
    labelAndSliderLayout->addRow(_autoWhiteBalanceCheckbox);

    // We add everything to the main layout
    formWidget->setLayout(labelAndSliderLayout);
    _mainLayout->addWidget(formWidget);

    _redBalance->setRange(1 * _dpi, 15 * _dpi);
    _greenBalance->setRange(1 * _dpi, 15 * _dpi);
    _blueBalance->setRange(1 * _dpi, 15 * _dpi);

    connect(_redBalance,
        &QSlider::valueChanged,
        this,
        &ParametersWidget::onWhiteBalanceChanged);
    connect(_greenBalance,
        &QSlider::valueChanged,
        this,
        &ParametersWidget::onWhiteBalanceChanged);
    connect(_blueBalance,
        &QSlider::valueChanged,
        this,
        &ParametersWidget::onWhiteBalanceChanged);

    connect(_autoWhiteBalanceCheckbox,
        &QCheckBox::stateChanged,
        this,
        &ParametersWidget::onAutoWhiteBalance);

}

void ParametersWidget::onGainChanged(int value)
{
    CameraState newState;
    newState.gain = float(value) / _dpi;
    std::stringstream s;
    s << newState.gain << " dB";
    _gainTicks->setText(s.str().c_str());
    emit updateParameters(GAIN, newState);
}

void ParametersWidget::onExposureChanged(double value)
{
    if (value != _lastExposureVal)
    {
        _lastExposureVal = value;
        CameraState newState;
        newState.exposureUs = _lastExposureVal;
        emit updateParameters(EXPOSURE_TIME, newState);
    }
}

void ParametersWidget::onAutoExposureChanged(int state)
{
    if (state != _lastExposureState)
    {
        _lastExposureState = state;
        CameraState newState;
        newState.autoExposureEnabled = state != Qt::Unchecked;
        _exposureControlWidget->setEnabled(!newState.autoExposureEnabled);
        emit updateParameters(AUTO_EXPOSURE, newState);
    }
    else
    {
        std::cout << "Trying to set the same previous state on AutoExposure" << std::endl;
    }
}

void ParametersWidget::onAutoGainChanged(int state)
{
    if (state != _lastGainState)
    {
        _lastGainState = state;
        CameraState newState;
        newState.autoGainEnabled = state != Qt::Unchecked;
        _gainControlWidget->setEnabled(!newState.autoGainEnabled);
        emit updateParameters(AUTO_GAIN, newState);
    }
    else
    {
        std::cout << "Trying to set the same previous state on AutoGain" << std::endl;
    }
}

void ParametersWidget::onFramerateChanged(int value)
{
    if (value != _lastFrameRate)
    {
        _lastFrameRate = value;
        CameraState newState;
        newState.frameRate = value;
        emit updateParameters(FRAME_RATE, newState);
    }
}

void ParametersWidget::onAutoWhiteBalance(int state)
{
    CameraState newState;
    newState.autoWhiteBalanceEnabled = state != Qt::Unchecked;

    _redBalance->setEnabled(!newState.autoWhiteBalanceEnabled);
    _greenBalance->setEnabled(!newState.autoWhiteBalanceEnabled);
    _blueBalance->setEnabled(!newState.autoWhiteBalanceEnabled);

    emit updateParameters(AUTO_WHITE_BALANCE, newState);
}

void ParametersWidget::onWhiteBalanceChanged(int value)
{
    (void)value;
    CameraState newState;
    newState.redGain = float(_redBalance->value()) / _dpi;
    newState.greenGain = float(_greenBalance->value()) / _dpi;
    newState.blueGain = float(_blueBalance->value()) / _dpi;
    std::stringstream s;

    s << newState.redGain;
    _redGainTicks->setText(s.str().c_str());
    s = std::stringstream();

    s << newState.greenGain;
    _greenGainTicks->setText(s.str().c_str());
    s = std::stringstream();

    s << newState.blueGain;
    _blueGainTicks->setText(s.str().c_str());

    emit updateParameters(WHITE_BALANCE_GAINS, newState);
}

void ParametersWidget::onNewStateReceived(const CameraState new_state)
{
    blockSignals(true);

    _gainControlWidget->setValue(static_cast<int>(new_state.gain * _dpi));
    _exposureControlWidget->setValue(new_state.exposureUs);
    _framerateControlWidget->setValue(new_state.frameRate);

    Qt::CheckState autoGainState = Qt::Unchecked;
    Qt::CheckState autoExpState = Qt::Unchecked;

    if (new_state.autoGainEnabled)
    {
        autoGainState = Qt::Checked;
    }

    if (new_state.autoExposureEnabled)
    {
        autoExpState = Qt::Checked;
    }
    _autoGainCheckbox->setCheckState(autoGainState);
    _autoExposureCheckbox->setCheckState(autoExpState);

    _redBalance->setValue(new_state.redGain * _dpi);
    _greenBalance->setValue(new_state.greenGain * _dpi);
    _blueBalance->setValue(new_state.blueGain * _dpi);

    blockSignals(false);
}

void ParametersWidget::setGainLimits(std::vector<int> l)
{
    _gainLimits = l;
    _gainControlWidget->setRange(_gainLimits[0] * _dpi, _gainLimits[1] * _dpi);
}

void ParametersWidget::setExposureLimits(std::vector<double> l)
{
    _exposureLimits = l;
    _exposureControlWidget->setMinimum(_exposureLimits[0]);
    _exposureControlWidget->setMaximum(_exposureLimits[1]);
}

void ParametersWidget::setSoftwareWhiteBalanceGains(std::vector<double> gains)
{
    blockSignals(true);
    _redBalance->setValue(gains[0] * _dpi);
    _greenBalance->setValue(gains[1] * _dpi);
    _blueBalance->setValue(gains[2] * _dpi);
    blockSignals(false);
}

void ParametersWidget::setAutoWhiteBalanceState(bool enabled)
{
    blockSignals(true);
    if (enabled)
    {
        _autoWhiteBalanceCheckbox->setCheckState(Qt::Checked);
    }
    else
    {
        _autoWhiteBalanceCheckbox->setCheckState(Qt::Unchecked);
    }
    blockSignals(false);
}