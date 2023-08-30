// OpenCV includes

// STD includes
#include <iostream>

// Qt Includes
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QInputDialog>
#include <QVBoxLayout>

// Pylon includes

// Custom includes
#include "LoadSaveWidget/LoadSaveWidget.hpp"

// ROS includes

LoadSaveWidget::LoadSaveWidget(std::vector<std::string> calibratorTypes, QWidget* parent) :
    QWidget(parent),
    _calibratorSelector(nullptr),
    _samplesSpinbox(nullptr),
    _angleStepSpinbox(nullptr),
    _storeDirectoryPath(nullptr),
    _newExpButton(nullptr),
    _storePath(std::string(PROJECT_PATH) + "captured_images/"),
    _prefixPathString("Storage folder: "),
    _pathLength(40),
    _calibratorTypes(calibratorTypes)
{
    initializeWidget();
}

LoadSaveWidget::~LoadSaveWidget()
{
    delete _newExpButton;
    delete _samplesSpinbox;
    delete _storeDirectoryPath;
    delete _angleStepSpinbox;
    delete _calibratorSelector;
}

void LoadSaveWidget::initializeWidget()
{
    QVBoxLayout* layout = new QVBoxLayout();
    // We create the three main buttons of the Save / Load Widget
    _newExpButton = new QPushButton("Create a new experiment", this);
    QPushButton* workingDirectoryFolder = new QPushButton("Select working directory", this);
    QPushButton* loadExp = new QPushButton("Load existing experiment", this);
    QPushButton* computeButton = new QPushButton("Compute pixel gain calibration");

    // List of calibrators QComboBox
    _calibratorSelector = new QComboBox(this);
    if (_calibratorTypes.empty())
    {
        _calibratorSelector->addItem("");
    }
    else
    {
        for (std::string& type : _calibratorTypes)
        {
            _calibratorSelector->addItem(type.c_str());
        }
    }

    // We create the spin box that holds the amount of samples per angle.
    _samplesSpinbox = new QSpinBox(this);
    QLabel* samplesLabel = new QLabel("Samples per angle:", this);
    _samplesSpinbox->setMinimum(1);
    _samplesSpinbox->setMaximum(1000000);
    _samplesSpinbox->setValue(10);

    // We create the spin box that holds the angle step between two set of samples.
    _angleStepSpinbox = new QSpinBox(this);
    QLabel* angleStepLabel = new QLabel("Angle step:", this);
    _angleStepSpinbox->setMinimum(1);
    _angleStepSpinbox->setMaximum(359);
    _angleStepSpinbox->setValue(5);

    // We create a single widget with the two spinboxes in a QFormLayout (label, widget).
    QFormLayout* form = new QFormLayout();
    QWidget* temp = new QWidget();
    form->addRow(samplesLabel, _samplesSpinbox);
    form->addRow(angleStepLabel, _angleStepSpinbox);
    temp->setLayout(form);

    // We create a label to show the current working directory.
    _storeDirectoryPath = new QLabel("", this);
    _storeDirectoryPath->setText((_prefixPathString + cropString(_storePath, _pathLength)).c_str());

    // We add the widgets into a layout
    layout->addWidget(_calibratorSelector);
    layout->addWidget(workingDirectoryFolder);
    layout->addWidget(_newExpButton);
    layout->addWidget(temp);

    layout->addWidget(loadExp);
    layout->addWidget(computeButton);
    layout->addWidget(_storeDirectoryPath);

    layout->addStretch(1);

    // We change the widget's layout with the one we created.
    setLayout(layout);

    // We add the connections
    connect(_newExpButton,
        &QPushButton::clicked,
        this,
        &LoadSaveWidget::onCreateData);

    connect(loadExp,
        &QPushButton::clicked,
        this,
        &LoadSaveWidget::onLoadPreviousExperiment);

    connect(workingDirectoryFolder,
        &QPushButton::clicked,
        this,
        &LoadSaveWidget::onChangeDirectory);

    connect(computeButton,
        &QPushButton::clicked,
        this,
        &LoadSaveWidget::computeCalibration);

    connect(_calibratorSelector,
        QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,
        &LoadSaveWidget::loaderChanged);
}

void LoadSaveWidget::setEnabled(bool enable)
{
    _newExpButton->setEnabled(enable);
    _samplesSpinbox->setEnabled(enable);
    _angleStepSpinbox->setEnabled(enable);
}

void LoadSaveWidget::onLoadPreviousExperiment()
{
    QStringList expList;
    // In order to have a list of options in all the cases, we need to add the
    // empty case. If not, the list becomes a QLineEdit
    expList << tr("");

    if (!_storePath.empty())
    {
        QDir rootDirectory(_storePath.c_str());
        // We check if the current working directory exists
        if (rootDirectory.exists())
        {
            // We read only the directories present in the working directory, and
            // we add them to the list to show
            rootDirectory.setFilter(QDir::Dirs | QDir::NoSymLinks | QDir::NoDot | QDir::NoDotDot);
            for (QFileInfo &item : rootDirectory.entryInfoList())
            {
                expList << item.baseName();
            }

            // The QInputDialog shows a QDialog with a list of options.
            bool ok;
            QString selectedExp = QInputDialog::getItem(this,
                tr("Select the experiment"),
                tr("Experiment:"),
                expList,
                0,
                false,
                &ok);

            // If the QInputDialog succeeded and the user selected an option that
            // is not the empty one, we emit the corresponding signal.
            if (ok && !selectedExp.isEmpty())
            {
                emit loadFiles(_storePath, selectedExp.toUtf8().toStdString());
            }
            else
            {
                std::cout << "No experiment entered" << std::endl;
            }
        }
        else
        {
            std::cout << "Directory does not exists" << std::endl;
        }
    }
    else
    {
        std::cout << "ERROR: The experiments folder path is empty" << std::endl;
    }
}

void LoadSaveWidget::onCreateData()
{
    if (!_storePath.empty())
    {
        bool ok;
        QString enteredText = QInputDialog::getText(this,
            tr("Enter an experiment name"),
            tr("Name:"),
            QLineEdit::Normal,
            "pixels_test",
            &ok);

        if (ok && !enteredText.isEmpty())
        {
            emit createExperiment(_samplesSpinbox->value(), _angleStepSpinbox->value(), _storePath, enteredText.toUtf8().toStdString());
        }
    }
    else
    {
        std::cout << "ERROR: The experiments folder path is empty" << std::endl;
    }
}

void LoadSaveWidget::onChangeDirectory()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);
    if(dialog.exec())
    {
        _storePath = dialog.directory().absolutePath().toUtf8().constData();
        _storeDirectoryPath->setText((_prefixPathString + cropString(_storePath, _pathLength)).c_str());
    }
    else
    {
        std::cout << "No path value entered." << std::endl;
    }
}

std::string LoadSaveWidget::cropString(std::string src, unsigned int desired_length)
{
    std::string output = src;
    if (src.size() > desired_length)
    {
        output = std::string("...") + src.substr(src.size() - desired_length + 3, desired_length - 3);
    }
    return output;
}