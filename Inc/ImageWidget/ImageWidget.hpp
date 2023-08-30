#ifndef __IMAGE_WIDGET_HPP__
#define __IMAGE_WIDGET_HPP__
// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes

// Qt Includes
#include <QPaintEvent>
#include <QWidget>

// Custom includes

/**
 * @brief ImageWidget class.
 *  This class shows a given image, defined by an OpenCV matrix type cv::Mat
 * inside a QWidget object. This eases showing the image into another QWidget
 * by just adding it into its layout.
*/
class ImageWidget : public QWidget
{
public:
    /**
     * @brief Constructor. It will initialize the widget structure.
     *
     * @arg parent: Parent Widget. Set to zero by default.
    */
    ImageWidget(QWidget *parent = 0);
    ~ImageWidget() = default;

    /**
     * @brief updateImage: Update the shown image. The image will be rescaled
     *      by a scaling factor computed in order to have a fixed image size,
     *      defined on construction. By default, this size is a width of 480 pixels.
     *      This way, the widget does not take a huge amount of space.
     *
     * @arg: new_image. An OpenCV image type cv::Mat with the new image to be
     *      shown. If the image is not empty, the update will be triggered and
     *      the repaint method will be called. A copy of this new image will be
     *      stored in this class.
    */
    void updateImage(const cv::Mat &new_image);

    /**
     * @brief clearBuffers: It empties the stored image (curently shown), and it
     *      will trigger a new repaint of the widget. This will blank the shown
     *      image.
    */
    void clearBuffers();

    /**
     * @brief updateImageWidth: Update the fixed width of the shown images.
     *      Each time we show an image, we scale it so they have the same width
     *      always.
     *
     * @arg new_width: New width to resize the images.
    */
    void updateImageWidth(int new_width);

    /**
     * @brief getImageWidth: Get the set image width. This value is the width
     *  that all the shown images will have.
     *
     * @returns: Integer with the fixed width value.
    */
    int getImageWidth() const {return _desired_width;}

protected:
    /**
     * @brief paintEvent: Overrided function. It will be called automatically
     *      when a repaint is scheduled. This function has been reimplemented
     *      in order to paint the stored image in the widget.
    */
    void paintEvent(QPaintEvent *event) override;

private:
    int _desired_width;
    cv::Mat _showedImage;
};
#endif // __IMAGE_WIDGET_HPP__
