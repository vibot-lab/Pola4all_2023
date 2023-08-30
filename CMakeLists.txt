cmake_minimum_required(VERSION 3.0.2)
project("Camera basler custom GUI app")

# Qt Configuration
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

# Python3 Configuration
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# OpenCV Configuration
find_package(OpenCV 4 REQUIRED)

set(PROGRAM_NAME Pola4All)
# In case of Debugging with QT Creator, uncomment these lines
# set(COMPILE_USB True)
# set(COMPILE_ROS True)
# set(CMAKE_BUILD_TYPE Debug)
# set(QMAKE_CXXFLAGS "-fno-sized-deallocation")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -std=c++0x -Wall -Wextra")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CPP_FILES
    main.cpp
    mainwindow.cpp
    Src/FilterOrientationsWidget/FilterOrientationsWidget.cpp
    Src/LoadSaveWidget/LoadSaveWidget.cpp
    Src/ImageWidget/ImageGridWidget.cpp
    Src/ImageWidget/ImagesConverter.cpp
    Src/ImageWidget/ImageWidget.cpp
    Src/MatPlotLibPlotsWidget/MatPlotLibPlotsWidget.cpp
    Src/ParametersWidget/ParametersWidget.cpp
    Src/PolarimetricCamera/LoadSaveExperiments/LoadSaveExperiments.cpp
    Src/PolarimetricCamera/PixelGainCalibration/BlackCurrentCorrection.cpp
    Src/PolarimetricCamera/PixelGainCalibration/CalibratorsHub.cpp
    Src/PolarimetricCamera/PixelGainCalibration/SourceOrientationEstimator.cpp
    Src/PolarimetricCamera/PixelGainCalibration/SuperPixelCalibration.cpp
    Src/PolarimetricCamera/PolarimetricImagesProcessing/PolarimetricImagesProcessing.cpp
    Src/RealTimeParameters/RealTimeParameters.cpp
    Src/PolarimetricCamera/RosCamClient/RosCamClient.cpp
    Src/PolarimetricCamera/RosCamClient/RosHandlerSingleton.cpp
    Src/SaveImagesWidget/SaveImagesWidget.cpp
    Src/SoftwareWhiteBalance/SoftwareWhiteBalance.cpp
    Src/VisualizationModeWidget/VisualizationModeWidget.cpp
)
set(HPP_FILES
    Inc/CameraTypes.hpp
    Inc/FilterOrientationsWidget/FilterOrientationsWidget.hpp
    Inc/LoadSaveWidget/LoadSaveWidget.hpp
    Inc/ImageWidget/ImageGridWidget.hpp
    Inc/ImageWidget/ImagesConverter.hpp
    Inc/ImageWidget/ImageWidget.hpp
    Inc/MatPlotLibPlotsWidget/matplotlib-cpp/matplotlibcpp.h
    Inc/MatPlotLibPlotsWidget/MatPlotLibPlotsWidget.hpp
    Inc/ParametersWidget/ParametersWidget.hpp
    Inc/PolarimetricCamera/LoadSaveExperiments/LoadSaveExperiments.hpp
    Inc/PolarimetricCamera/PixelGainCalibration/BlackCurrentCorrection.hpp
    Inc/PolarimetricCamera/PixelGainCalibration/CalibratorsHub.hpp
    Inc/PolarimetricCamera/PixelGainCalibration/SourceOrientationEstimator.hpp
    Inc/PolarimetricCamera/PixelGainCalibration/SuperPixelCalibration.hpp
    Inc/PolarimetricCamera/PolarimetricImagesProcessing/PolarimetricImagesProcessing.hpp
    Inc/RealTimeParameters/RealTimeParameters.hpp
    Inc/PolarimetricCamera/RosCamClient/RosCamClient.hpp
    Inc/PolarimetricCamera/RosCamClient/RosHandlerSingleton.hpp
    Inc/SaveImagesWidget/SaveImagesWidget.hpp
    Inc/SoftwareWhiteBalance/SoftwareWhiteBalance.hpp
    Inc/VisualizationModeWidget/VisualizationModeWidget.hpp
    mainwindow.h
)

set(ROS_DISTRIBUTION "noetic")
find_package(
    roscpp REQUIRED
    std_msgs REQUIRED
    catkin REQUIRED
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    RosCameraServer/devel/include
    RosCameraServer/src/vision_opencv/cv_bridge/include
    /opt/ros/${ROS_DISTRIBUTION}/include
)

link_directories(
    RosCameraServer/devel/lib
    /opt/ros/${ROS_DISTRIBUTION}/lib)

set(SOURCES
    ${CPP_FILES}
    ${HPP_FILES}
)

## We define a variable that will be available in the CPP program. This way
## we do not depend where the binary is installed.
add_definitions(-DPROJECT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/")

# Includes DIR of our project
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# qt5_wrap_cpp: Required for Qt5
qt5_wrap_cpp(SOURCES mainwindow.h)
add_executable(${PROGRAM_NAME} ${SOURCES})

# Common executable link libraries
target_link_libraries(${PROGRAM_NAME}
    ${PYTHON_LIBRARIES}
    stdc++fs
    opencv_core
    opencv_imgcodecs
    opencv_highgui
    opencv_videoio
    opencv_imgproc
    Qt5::Core
    Qt5::Network
    Qt5::Gui
    Qt5::Widgets
    roscpp
    rosconsole
    roscpp_serialization
    rostime
    cv_bridge
    image_transport
)