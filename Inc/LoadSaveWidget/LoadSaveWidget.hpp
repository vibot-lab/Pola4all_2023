#ifndef __LOAD_SAVE_WIDGET_HPP__
#define __LOAD_SAVE_WIDGET_HPP__

// OpenCV includes

// STD includes
#include <string>
#include <vector>

// Qt Includes
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QWidget>

// Pylon includes

// Custom includes

// ROS includes

/**
 * @brief class LoadSaveWidget: This class holds a widget to make the user
 * interface with the module to save and load camera experiments. This widget
 * includes one drop-down menu, three buttons, and two spin boxes. The drop-down
 * menu shows the available calibrators (pixel gain, black current, etc).
 * The buttons allow the user to create a new experiment, load a new experiment
 * and change the current working directory. The first spinbox allows the user
 * to change the amount of samples per polarization angle, and the second one
 * allows the user to change the polarization angle step.
 * This last spin box allows for instance, decide to take images every 10 degrees,
 * or 5 degrees, or any other number that the user wants, to reduce or increase
 * the cosine function resolution.
 *
 *  The experiment we deal in this case is the pixel gains. We place the polarization
 * camera in front of a linearly polarized light, with a known angle. We take
 * several samples and we do the average of them. Then, we change the angle
 * a value equal to the angle step, and we do the same. We repeat this procedure
 * as many times as we want. The idea is that, per pixel, we will have a set
 * of measurements, that should describe a shifted cosine function.
 *
 *  This widget allows the user to save and load the data for that experiment.
*/
class LoadSaveWidget : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     *
     * @arg calibratorTypes: Vector with the names of the calibrators
    */
    LoadSaveWidget(std::vector<std::string> calibratorTypes, QWidget* parent = 0);
    ~LoadSaveWidget();

    /**
     * @brief setEnabled: Enable / disable all the internal widgetsof this class
     *
     * @arg enable: If true, all the widgets are enabled. They are disabled if this
     *  variable is false.
    */
    void setEnabled(bool enable);

    /// \brief getSelectedKey: Get the string with the selected calibrator name
    std::string getSelectedKey() {return _calibratorSelector->currentText().toStdString();}

protected:
    /**
     * @brief onLoadPreviousExperiment: This slot is called each time the button
     * "Load previous experiment" is pressed. Using the current workspace path,
     * it will look for directories in this folder, and it will show them as a list.
     *  Then, when the user choose one of the options, this function will emit
     * the corresponding signal, with the working directory path, and the chosen
     * folder name.
    */
    void onLoadPreviousExperiment();

    /**
     * @brief onCreateData: Slot called each time the user presses the button
     * "Create new experiment". It will show a dialog to the user, to enter a
     * name for the experiment. If the entered string is not empty, it will emit
     * a signal with the current amount of samples, the current angle step,
     * the working path, and the experiment name.
    */
    void onCreateData();

    /**
     * @brief onChangeDirectory: It will show a dialog to select a new working
     * directory. The result of this operation will be stored locally in the class,
     * and shown in a label.
    */
    void onChangeDirectory();

signals:
    /**
     * @brief createExperiment: Signal emitted each time the user wants to create
     * a new experiment. This signal is emitted only if the user entered all
     * the required information.
     *
     * @arg samples: Amount of samples to take for each angle.
     * @arg angleStep: Step to increase the polarization angle, after taking
     *  "samples" amount of samples.
     * @arg path: Current working directory
     * @arg expName: Experiment name.
    */
    void createExperiment(int samples, int angleStep, std::string path, std::string expName);

    /**
     * @brief loadFiles: Signal emitted each time the user wants to load an existing
     *  experiment.
     *
     * @arg path: Working directory absolute path.
     * @arg expName: Experiment name.
    */
    void loadFiles(std::string path, std::string expName);

    /**
     * @brief computeCalibration: Signal emitted each time the button
     * "Compute pixel gain" is pressed
    */
    void computeCalibration();

    /**
     * @brief loaderChanged: Signal emitted each time the user changes the value
     * in the QComboBox that corresponds to the calibrator selector.
    */
    void loaderChanged();

private:
    /**
     * @brief initializeWidget: Initialize the widgets that are part of this
     *  custom widget, and its layout.
    */
    void initializeWidget();

    /**
     * @brief cropString: Function that reduces the size of the filepath
     *  in order to show it properly in the GUI, and avoid that this will
     *  take too much space that it will make the right pannel to grow too much.
     *
     * @arg src: String to be reduced in size.
     * @arg desired_length: It specifies how many elements the string should have.
     *
     * @return std::string with the reduced size string. The beginning of it
     *  will start with three dots (...) in order to make the user to understand
     *  the string is actually longer than the showed text.
    */
    std::string cropString(std::string src, unsigned int desired_length);

    QComboBox* _calibratorSelector;

    QSpinBox* _samplesSpinbox;
    QSpinBox* _angleStepSpinbox;

    QLabel* _storeDirectoryPath;

    QPushButton* _newExpButton;

    std::string _storePath;
    std::string _prefixPathString;

    int _pathLength;
    std::vector<std::string> _calibratorTypes;
};

#endif // __LOAD_SAVE_WIDGET_HPP__