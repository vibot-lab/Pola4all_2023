// OpenCV includes
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// STD includes
#include <iostream>

// Qt Includes
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

// Custom includes
#include "SaveImagesWidget/SaveImagesWidget.hpp"

SaveImagesWidget::SaveImagesWidget(QWidget* parent) :
    QWidget(parent),
    _storePath(""),
    _storeDirectoryPath(nullptr),
    _pathLength(30)
{
    initializeWidget();
}

SaveImagesWidget::~SaveImagesWidget()
{
    delete _storeDirectoryPath;
}

std::string SaveImagesWidget::cropString(std::string src, unsigned int desired_length)
{
    std::string output = src;
    if (src.size() > desired_length)
    {
        output = std::string("...") + src.substr(src.size() - desired_length + 3, desired_length - 3);
    }
    return output;
}

void SaveImagesWidget::initializeWidget()
{
    QVBoxLayout* layout = new QVBoxLayout();
    QPushButton* changeDirButton = new QPushButton("Change store directory", this);
    QPushButton* storeImageButton = new QPushButton("Store image", this);
    QWidget* pathWidget = new QWidget(this);
    QFormLayout* pathLayout = new QFormLayout(pathWidget);

    // The PROJECT_PATH macro is defined during compilation time
    _storePath = std::string(PROJECT_PATH) + "captured_images/";
    QLabel* pathTitleLabel = new QLabel("Storage folder:", pathWidget);
    _storeDirectoryPath = new QLabel("", pathWidget);

    pathLayout->addRow(pathTitleLabel, _storeDirectoryPath);
    pathWidget->setLayout(pathLayout);

    layout->addWidget(changeDirButton);
    layout->addWidget(storeImageButton);
    layout->addWidget(pathWidget);

    setLayout(layout);

    _storeDirectoryPath->setText(cropString(_storePath, _pathLength).c_str());

    connect(changeDirButton,
        &QPushButton::clicked,
        this,
        &SaveImagesWidget::onChangeDirectory);

    connect(storeImageButton,
        &QPushButton::clicked,
        this,
        &SaveImagesWidget::onStoreImage);
}

void SaveImagesWidget::onStoreImage()
{
    if (!_storePath.empty())
    {
        QDir myDir(_storePath.c_str());
        if (myDir.mkpath(_storePath.c_str()))
        {
            emit requestCaptureImages();
        }
        else
        {
            std::cout << "Cannot create directory " << _storePath << ". Not storing the images" << std::endl;
        }
    }
    else
    {
        std::cout << "Store directory not provided. Not storing the images" << std::endl;
    }
}

void SaveImagesWidget::onChangeDirectory()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);
    if(dialog.exec())
    {
        _storePath = dialog.directory().absolutePath().toUtf8().constData();
        _storeDirectoryPath->setText(cropString(_storePath, _pathLength).c_str());
    }
    else
    {
        std::cout << "No path value entered." << std::endl;
    }
}

void SaveImagesWidget::onImagesReceived(std::vector<cv::Mat> &images)
{
    bool ok;
    std::string text = QInputDialog::getText(this, tr("Save polarized images"),
                                         tr("Please enter a filename prefix:"), QLineEdit::Normal,
                                         "polarized_images", &ok).toStdString();
    if (ok)
    {
        if (!text.empty())
        {
            QDir myDir(_storePath.c_str());
            std::string prefixPath = myDir.filePath(
                (text +
                QDateTime::currentDateTime().toString("_ddMMyyyy_hhmmss").toUtf8().constData()).c_str()
            ).toUtf8().constData();

            // The images are actually switched in R and B. So we need to invert them
            // from BGR to RGB in order to have the right tint
            for (unsigned int i = 0; i < _labels.size(); i++)
            {
                std::string filename = prefixPath + "_" + _labels[i] + ".png";
                cv::Mat output = images[i];
                if (images[i].channels() != 1)
                {
                    cv::cvtColor(images[i], output, cv::COLOR_BGR2RGB);
                }
                cv::imwrite(filename.c_str(), output);
            }

            QMessageBox msgBox;
            msgBox.setWindowTitle("Save polarized images");
            msgBox.setIcon(QMessageBox::Information);
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.setText(QString("Images stored successfully at ") + _storePath.c_str());
            msgBox.exec();
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setWindowTitle("Save polarized images");
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.setStandardButtons(QMessageBox::Ok);
            msgBox.setText("You must provide a prefix for your pictures! Images not stored");
            msgBox.exec();
        }
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Save polarized images");
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setText("Images not stored");
        msgBox.exec();
    }
}