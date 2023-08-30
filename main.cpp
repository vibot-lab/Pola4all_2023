// OpenCV includes

// STD includes

// Qt Includes
#include <QApplication>

// Custom includes
#include "CameraTypes.hpp"
#include "mainwindow.h"

// ROS includes

int main(int argc, char *argv[])
{
    /// Since there is another thread in the camera implementation,
    // we need to register this type in order to avoid signal / slot problems
    qRegisterMetaType<CameraState>("CameraState");

    // std::shared_ptr<bitDepthStruct> usedFormat(new bitDepthStruct_8u());
    std::shared_ptr<bitDepthStruct> usedFormat(new bitDepthStruct_12u());

    QApplication a(argc, argv);
    MainWindow w(usedFormat);
    w.show();
    return a.exec();
}
