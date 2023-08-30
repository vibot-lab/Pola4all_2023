
// OpenCV includes

// STD includes
#include <iostream>

// Qt Includes
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

// Custom includes
#include "VisualizationModeWidget/VisualizationModeWidget.hpp"

VisualizationModeWidget::VisualizationModeWidget(QWidget* parent) :
    QWidget(parent),
    _processingListComboBox(nullptr),
    _aopSelector(nullptr)
{
    initializeLabels({135, 0, 90, 45});
    initializeWidget();
}

VisualizationModeWidget::~VisualizationModeWidget()
{
    delete _processingListComboBox;
    delete _aopSelector;
}

void VisualizationModeWidget::initializeWidget()
{
    // Initial mode
    std::string firstSelection = "Polarized color images";

    QVBoxLayout* layout = new QVBoxLayout(this);
    _processingListComboBox = new QComboBox(this);
    for(std::pair<std::string, ProcessingLevel> imageType : _fromSelectionToProcessing)
    {
        _processingListComboBox->addItem(tr(imageType.first.c_str()));
    }
    layout->addWidget(_processingListComboBox);

    // We add the AoLP spinbox and its text
    QWidget* temp = new QWidget(this);
    QHBoxLayout* spinBoxForm = new QHBoxLayout(temp);
    QLabel* text = new QLabel("AoLP: ", this);
    _aopSelector = new QSpinBox(this);
    _aopSelector->setMinimum(0);
    _aopSelector->setMaximum(359);
    spinBoxForm->addWidget(text);
    spinBoxForm->addWidget(_aopSelector);
    temp->setLayout(spinBoxForm);
    layout->addWidget(temp);

    int idx = _processingListComboBox->findText(firstSelection.c_str(), Qt::MatchContains);
    if (idx != -1)
    {
        _processingListComboBox->setCurrentIndex(idx);
        std::string currentTxt = _processingListComboBox->currentText().toUtf8().toStdString();
        _selectedProcessing = _fromSelectionToProcessing.at(currentTxt);
        _selectedLabels = _labelsMap.at(_selectedProcessing);
        _aopSelector->setEnabled(false);
    }
    else
    {
        std::cerr << "Initial mode not found" << std::endl;
        assert(0);
    }

    connect(
        _processingListComboBox,
        &QComboBox::currentTextChanged,
        this,
        &VisualizationModeWidget::onProcessingLevelChanged);
}

void VisualizationModeWidget::onProcessingLevelChanged(const QString &text)
{
    _selectedProcessing = _fromSelectionToProcessing.at(text.toStdString());
    _selectedLabels = _labelsMap.at(_selectedProcessing);

    // The spinbox is valid only in one case.
    if (_selectedProcessing == SIM_POLARIZER)
    {
        _aopSelector->setEnabled(true);
    }
    else
    {
        _aopSelector->setEnabled(false);
    }

    emit modeChanged();
}

void VisualizationModeWidget::initializeLabels(std::vector<int> filterOrientations)
{
    // Labels for RAW_IMAGE case
    _labelsMap[RAW_IMAGE] = std::vector<std::string> ({
        "raw_image"
    });

    // Labels for RAW_SPLITTED_IMAGES case
    _labelsMap[RAW_SPLITTED_IMAGES] = std::vector<std::string> ({
        "raw_" + std::to_string(filterOrientations[0]),
        "raw_" + std::to_string(filterOrientations[1]),
        "raw_" + std::to_string(filterOrientations[2]),
        "raw_" + std::to_string(filterOrientations[3])
    });

    // Labels for COLOR_SPLITTED case
    _labelsMap[COLOR_SPLITTED] = std::vector<std::string> ({
        "color_" + std::to_string(filterOrientations[0]),
        "color_" + std::to_string(filterOrientations[1]),
        "color_" + std::to_string(filterOrientations[2]),
        "color_" + std::to_string(filterOrientations[3])
    });

    // Labels for COLOR_ORIGINAL case
    _labelsMap[COLOR_ORIGINAL] = std::vector<std::string> ({
        "original_color"
    });

    // Labels for SIM_POLARIZER case
    _labelsMap[SIM_POLARIZER] = std::vector<std::string> ({
        "original_color",
        "simulated_polarized_color"
    });

    // Labels for FAKE_COLORS case
    _labelsMap[FAKE_COLORS] = std::vector<std::string> ({
        "fake_color_red",
        "fake_color_green_1",
        "fake_color_green_2",
        "fake_color_blue"
    });

    // Labels for REMOVE_SPECULARITY case
    _labelsMap[REMOVE_SPECULARITY] = std::vector<std::string> ({
        "original_color",
        "filtered_image"
    });

    // Labels for the STOKES case
    _labelsMap[STOKES] = std::vector<std::string> ({
        "stokes_s0_red",
        "stokes_s0_green_1",
        "stokes_s0_green_2",
        "stokes_s0_blue",
        "stokes_s1_red",
        "stokes_s1_green_1",
        "stokes_s1_green_2",
        "stokes_s1_blue",
        "stokes_s2_red",
        "stokes_s2_green_1",
        "stokes_s2_green_2",
        "stokes_s2_blue"
    });

    // Labels for the I_RO_PHI case
    _labelsMap[I_RO_PHI] = std::vector<std::string> ({
        "intensity_red",
        "intensity_green_1",
        "intensity_green_2",
        "intensity_blue",
        "angle_of_polarization_red",
        "angle_of_polarization_green_1",
        "angle_of_polarization_green_2",
        "angle_of_polarization_blue",
        "degree_of_polarization_red",
        "degree_of_polarization_green_1",
        "degree_of_polarization_green_2",
        "degree_of_polarization_blue"
    });

    // Labels for the RAW_I_RO_PHI case
    _labelsMap[RAW_I_RO_PHI] = _labelsMap[I_RO_PHI];

    _fromSelectionToProcessing["01 - Raw image"] = RAW_IMAGE;
    _fromSelectionToProcessing["02 - Raw splitted images"] = RAW_SPLITTED_IMAGES;
    _fromSelectionToProcessing["03 - Polarized color images"] = COLOR_SPLITTED;
    _fromSelectionToProcessing["04 - Original color"] = COLOR_ORIGINAL;
    _fromSelectionToProcessing["05 - Simulated polarizer"] = SIM_POLARIZER;
    _fromSelectionToProcessing["06 - Stokes images"] = STOKES;
    _fromSelectionToProcessing["07 - I - Ro - Phi"] = I_RO_PHI;
    _fromSelectionToProcessing["08 - Raw I - Ro - Phi"] = RAW_I_RO_PHI;
    _fromSelectionToProcessing["09 - Fake colors"] = FAKE_COLORS;
    _fromSelectionToProcessing["10 - Remove specularity"] = REMOVE_SPECULARITY;

    // NOTE If this assert if false, then the amount of processing cases,
    // and the definitions in the map are not the same. Check if there is either
    // a case that has been erased, or a new processing level added.
    assert(_fromSelectionToProcessing.size() == NUM_PROCESSING_OPT);
}

void VisualizationModeWidget::updateAngleLabels(std::vector<int> newOrient)
{
    // Labels for RAW_SPLITTED_IMAGES case
    _labelsMap[RAW_SPLITTED_IMAGES] = std::vector<std::string> ({
        "raw_" + std::to_string(newOrient[0]),
        "raw_" + std::to_string(newOrient[1]),
        "raw_" + std::to_string(newOrient[2]),
        "raw_" + std::to_string(newOrient[3])
    });

    // Labels for COLOR_SPLITTED case
    _labelsMap[COLOR_SPLITTED] = std::vector<std::string> ({
        "color_" + std::to_string(newOrient[0]),
        "color_" + std::to_string(newOrient[1]),
        "color_" + std::to_string(newOrient[2]),
        "color_" + std::to_string(newOrient[3])
    });
    _selectedLabels = _labelsMap.at(_selectedProcessing);
}