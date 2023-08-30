// OpenCV includes
#include "opencv2/imgproc/imgproc.hpp"

// STD includes
#include <iostream>
#include <mutex>

// Qt Includes
#include <QImage>
#include <QPainter>

// Custom includes
#include "ImageWidget/ImagesConverter.hpp"
#include "ImageWidget/ImageWidget.hpp"

static std::mutex _imgMtx;

ImageWidget::ImageWidget(QWidget *parent) :
    QWidget(parent),
    _desired_width(480)
{
}

void ImageWidget::updateImage(const cv::Mat &new_image)
{
    if (!new_image.empty())
    {
        float scalingFactor = float(_desired_width) / new_image.cols;
        {
            std::unique_lock<std::mutex> lck(_imgMtx);
            cv::resize(new_image, _showedImage, cv::Size(), scalingFactor, scalingFactor, cv::INTER_LINEAR);
            setFixedSize(
                _showedImage.cols,
                _showedImage.rows);
        }

        repaint();
    }
    else
    {
        std::cout << "Not updating image. The provided image is empty" << std::endl;
    }
}

void ImageWidget::updateImageWidth(int new_width)
{
    if (new_width >= 2)
    {
        _desired_width = new_width;
    }
    else
    {
        std::cout << "ERROR: You cannot set the image width to be less than 2" << std::endl;
    }
}

void ImageWidget::clearBuffers()
{
    _showedImage = cv::Mat();
    repaint();
}

void ImageWidget::paintEvent(QPaintEvent *event)
{
    (void)event;
    QImage img;
    {
        std::unique_lock<std::mutex> lck(_imgMtx);
        img = ImagesConverter::cvMatToQImage(_showedImage);
    }

    QPainter painter(this);
    painter.drawImage(
        0,
        0,
        img);
}
