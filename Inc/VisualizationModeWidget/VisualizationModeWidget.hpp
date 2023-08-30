#ifndef __VISUALIZATION_MODE_WIDGET_HPP__
#define __VISUALIZATION_MODE_WIDGET_HPP__

// OpenCV includes

// STD includes
#include <string>
#include <vector>

// Qt Includes
#include <QComboBox>
#include <QSpinBox>
#include <QString>
#include <QWidget>

// Custom includes
#include <CameraTypes.hpp>

/**
 * @brief VisualizationModeWidget: This widget holds the processing level labels,
 * and the QComboBox that allows to change from one more to another one.
 *  The different labels are hardcoded into this module, and the selected ones
 * can be queried when required. Each time the mode is changed, the variables
 * that hold these labels are updated.
 *  As part of the module functionality, a spinbox is added to select the AoLP
 * of the filter used for the SIM_POLARIZER case.
*/
class VisualizationModeWidget : public QWidget
{
    Q_OBJECT

public:
    /// \brief Constructor
    VisualizationModeWidget(QWidget* parent = 0);
    ~VisualizationModeWidget();

    /**
     * @brief getSelectedLabels: Get the set of labels that corresponds
     * to the selected mode. These labels identify the images obtained after
     * processing. This way, when showing them, or when storing them, it is
     * possible to identify the meaning of the image. The labels change as we
     * change the processing level.
     *
     * @returns: Vector of strings. Each element of this vector contains a phrase,
     * separated by under-scores, that identifies the content of the image.
    */
    std::vector<std::string>& getSelectedLabels() {return _selectedLabels;}

    /// \brief getSelectedMode: Get the selected mode identifier.
    ProcessingLevel getSelectedMode() const {return _selectedProcessing;}

    /// \brief getAoLP: Get the current AoLP selected in the SpinBox, in degrees.
    int getAoLP() const {return _aopSelector->value();}

    /**
     * @brief updateAngleLabels: Update the label names that are based on the
     *   filter orientations.
     *
     * @arg newOrient: Vector with the four orientations of the super-pixel,
     *  given in order from top to bottom and from left to right. The
     *  angles must be given in degrees.
    */
    void updateAngleLabels(std::vector<int> newOrient);

protected slots:
    /**
     * @brief onProcessingLevelChanged: Slot executed each time a new entry of
     * the combo box is selected. This combo box is the one that holds the names
     * of the different processing levels. This slot updates the variables that
     * hold the label values and the selected mode. It also emits a signal to
     * communicate the parent about this change.
     *
     * @arg text: Text of the new entry.
    */
    void onProcessingLevelChanged(const QString &text);

signals:
    /// \brief modeChanged: Signal emitted each time the processing level changes.
    void modeChanged();

private:
    /**
     * @brief initializeLabels: Initialize the internal map of image labels.
     *   These labels are used to identify which image we are being seeing, and
     *  they depend on the processing level selected. For instance, if we
     *  have a processing level "Polarized color images", we need to know
     *  that the top left image is the color image with an angle of
     *  linear polarization 135 degrees. The same when we store an image in disk.
     *   This function initializes the maps that holds all the possible labels
     *  for all the possible cases.
     *
     * @arg filterOrientations: Super-pixel filters orientations, read from top
     *  to the bottom, and from left to right. The angles must be given in degrees.
    */
    void initializeLabels(std::vector<int> filterOrientations);

    /// \brief initializeWidget: Initialize the internal widgets of the module.
    void initializeWidget();

    /// Internal variables
    QComboBox* _processingListComboBox;
    QSpinBox* _aopSelector;

    /// Maps that allows to know which processing level we use, and its labels
    ProcessingLevel _selectedProcessing;
    std::vector<std::string> _selectedLabels;
    std::map<ProcessingLevel, std::vector<std::string>> _labelsMap;
    std::map<std::string, ProcessingLevel> _fromSelectionToProcessing;
};
#endif // __VISUALIZATION_MODE_WIDGET_HPP__