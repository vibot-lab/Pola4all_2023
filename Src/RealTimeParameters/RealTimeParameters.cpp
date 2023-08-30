// OpenCV includes
#include <opencv2/core.hpp>

// STD includes
#include <iomanip>

// Qt Includes
#include <QFont>
#include <QGroupBox>
#include <QVBoxLayout>

// Pylon includes

// Custom includes
#include "RealTimeParameters/RealTimeParameters.hpp"

RealTimeParameters::RealTimeParameters(std::shared_ptr<bitDepthStruct> camImgFormat, QWidget* parent) :
    QWidget(parent),
    _aopLabel(nullptr),
    _maxValLabel(nullptr),
    _minValLabel(nullptr),
    _minDoLPLabel(nullptr),
    _meanDoLPLabel(nullptr),
    _maxDoLPLabel(nullptr),
    _paramsEnableCheckbox(nullptr),
    _camImgFormat(camImgFormat)
{
    initializeWidget();
}

RealTimeParameters::~RealTimeParameters()
{
    delete _minValLabel;
    delete _maxValLabel;
    delete _aopLabel;
    delete _minDoLPLabel;
    delete _meanDoLPLabel;
    delete _maxDoLPLabel;
    delete _paramsEnableCheckbox;
}

void RealTimeParameters::initializeWidget()
{
    QVBoxLayout* layout = new QVBoxLayout();
    QFont f("Arial", 14, QFont::Bold, true);

    QGroupBox* liveParamsGroup = new QGroupBox("Live parameters", this);
    QVBoxLayout* groupLayout = new QVBoxLayout(liveParamsGroup);
    _aopLabel = new QLabel("AoP: 0.0", liveParamsGroup);
    _maxValLabel = new QLabel("Max. intensity: 0.0", liveParamsGroup);
    _minValLabel = new QLabel("Min. intensity: 0.0", liveParamsGroup);
    _minDoLPLabel = new QLabel("Min. DoLP: 0.0", liveParamsGroup);
    _meanDoLPLabel = new QLabel("Mean DoLP: 0.0", liveParamsGroup);
    _maxDoLPLabel = new QLabel("Max. DoLP: 0.0", liveParamsGroup);
    _paramsEnableCheckbox = new QCheckBox("Enable", liveParamsGroup);

    _aopLabel->setFont(f);
    _minValLabel->setFont(f);
    _maxValLabel->setFont(f);
    _minDoLPLabel->setFont(f);
    _meanDoLPLabel->setFont(f);
    _maxDoLPLabel->setFont(f);

    bool initialState = false;
    _paramsEnableCheckbox->setChecked(initialState);
    _aopLabel->setEnabled(initialState);
    _minValLabel->setEnabled(initialState);
    _maxValLabel->setEnabled(initialState);
    _minDoLPLabel->setEnabled(initialState);
    _meanDoLPLabel->setEnabled(initialState);
    _maxDoLPLabel->setEnabled(initialState);

    groupLayout->addWidget(_paramsEnableCheckbox);
    groupLayout->addWidget(_aopLabel);
    groupLayout->addWidget(_minValLabel);
    groupLayout->addWidget(_maxValLabel);
    groupLayout->addWidget(_minDoLPLabel);
    groupLayout->addWidget(_meanDoLPLabel);
    groupLayout->addWidget(_maxDoLPLabel);
    liveParamsGroup->setLayout(groupLayout);

    layout->addWidget(liveParamsGroup);

    setLayout(layout);

    connect(
        _paramsEnableCheckbox,
        &QCheckBox::stateChanged,
        this,
        &RealTimeParameters::onChangeParamsCheckbox);
}

std::string RealTimeParameters::createStringWithNumber(std::string txt, double number, int precision)
{
    std::ostringstream sstream;
    sstream << txt  << std::fixed << std::setprecision(precision) << number;
    return sstream.str();
}

void RealTimeParameters::updateParamLabels(ParametersSet params)
{
    _minValLabel->setText(createStringWithNumber("Min. intensity: ", params.minInt, 0).c_str());
    _maxValLabel->setText(createStringWithNumber("Max. intensity: ", params.maxInt, 0).c_str());
    _aopLabel->setText(createStringWithNumber("AoP: ", params.aolp, 2).c_str());
    _minDoLPLabel->setText(createStringWithNumber("Min. DoLP: ",  params.minDoLP / _camImgFormat->_maxAllowedVal, 3).c_str());
    _meanDoLPLabel->setText(createStringWithNumber("Mean DoLP: ", params.meanDoLP / _camImgFormat->_maxAllowedVal, 3).c_str());
    _maxDoLPLabel->setText(createStringWithNumber("Max. DoLP: ",  params.maxDoLP / _camImgFormat->_maxAllowedVal, 3).c_str());
}

void RealTimeParameters::onChangeParamsCheckbox(int state)
{
    bool enabled = (state == Qt::Checked);
    if (!enabled)
    {
        _aopLabel->setText("AoP: 0.0");
        _maxValLabel->setText("Max. intensity: 0.0");
        _minValLabel->setText("Min. intensity: 0.0");
        _minDoLPLabel->setText("Min. DoLP: 0.0");
        _meanDoLPLabel->setText("Mean DoLP: 0.0");
        _maxDoLPLabel->setText("Max. DoLP: 0.0");
    }
    _aopLabel->setEnabled(enabled);
    _minValLabel->setEnabled(enabled);
    _maxValLabel->setEnabled(enabled);
    _minDoLPLabel->setEnabled(enabled);
    _meanDoLPLabel->setEnabled(enabled);
    _maxDoLPLabel->setEnabled(enabled);
}