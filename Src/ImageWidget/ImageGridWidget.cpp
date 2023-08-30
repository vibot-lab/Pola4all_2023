// OpenCV includes

// STD includes
#include <cassert>
#include <vector>

// Qt Includes

// Custom includes
#include "ImageWidget/ImageGridWidget.hpp"

ImageGridWidget::ImageGridWidget(QWidget* parent) :
    QWidget(parent),
    _nb_widgets(12),
    _nb_cols(4),
    _layout(new QGridLayout(this))
{
    initializeWidget();
    clearBuffers();
}

ImageGridWidget::~ImageGridWidget()
{
    delete _layout;
    for (unsigned int i = 0; i < _nb_widgets; i++)
    {
        delete _labelWidgets[i];
        delete _images[i];
    }
}

void ImageGridWidget::initializeWidget()
{
    for (unsigned int i = 0; i < _nb_widgets; i++)
    {
        int row = 2 * static_cast<int>(i / _nb_cols);
        int col = i % _nb_cols;
        ImageWidget* imgWidget = new ImageWidget(this);
        QLabel* imageIdLabel = new QLabel("", this);
        imageIdLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        _labelWidgets.push_back(imageIdLabel);
        _images.push_back(imgWidget);

        _layout->addWidget(imgWidget, row, col);
        _layout->addWidget(imageIdLabel, row + 1, col, Qt::AlignCenter);
    }
    _layout->setSizeConstraint(QLayout::SetFixedSize);

    setLayout(_layout);
}

void ImageGridWidget::updateGridImages(std::vector<cv::Mat> &new_images, int colLimit)
{
    unsigned int effectiveAmountWidgets = (float(_nb_widgets) / _nb_cols) * colLimit;
    assert(new_images.size() <= effectiveAmountWidgets);
    assert(colLimit > 0 && colLimit < 5);

    unsigned int imgCounter = 0;
    for (unsigned int i = 0; i < _nb_widgets; i++)
    {
        int col = i % _nb_cols;
        if ((col < colLimit) && imgCounter < new_images.size() && !new_images[imgCounter].empty())
        {
            _images[i]->updateImage(new_images[imgCounter]);
            _labelWidgets[i]->setText(("Image: " + _imageLabels[imgCounter]).c_str());
            setWidgetState(i, true);
            imgCounter++;
        }
        else
        {
            setWidgetState(i, false);
        }
    }
}

void ImageGridWidget::clearBuffers()
{
    for (unsigned int i = 0; i < _nb_widgets; i++)
    {
        _images[i]->clearBuffers();
        _labelWidgets[i]->setText("");
        setWidgetState(i, false);
    }
}

void ImageGridWidget::setWidgetState(int id, bool enabled)
{
    assert(id >= 0 && id < (int)_images.size());
    if (enabled)
    {
        _labelWidgets[id]->show();
        _images[id]->show();
    }
    else
    {
        _labelWidgets[id]->hide();
        _images[id]->hide();
    }
}

int ImageGridWidget::getImageWidth() const
{
    if (_images.size())
    {
        return _images[0]->getImageWidth();
    }
    else
    {
        return -1;
    }
}

void ImageGridWidget::changeImageWidth(int new_width)
{
    for (unsigned int i = 0; i < _nb_widgets; i++)
    {
        _images[i]->updateImageWidth(new_width);
    }
}
