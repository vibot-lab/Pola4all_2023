#ifndef __IMAGE_GRID_WIDGET_HPP__
#define __IMAGE_GRID_WIDGET_HPP__
// OpenCV includes

// STD includes
#include <string>
#include <vector>

// Qt Includes
#include <QGridLayout>
#include <QLabel>
#include <QWidget>

// Custom includes
#include "ImageWidget/ImageWidget.hpp"

/**
 * @brief ImageGridWidget Class
 * This class is a QWidget that groups several ImageWidgets
 * The idea of this little wrapper is to manage a vector of ImageWidgets in
 * an easy way. This is of particular use for the polarimetric camera, since
 * each single image contains 4 different polarization images.
 *
 *  The widgets are disponsed in a matrix shape. The way to define the matrix shape
 * is through the parameter _nb_cols (number of columns). For instance, if we have
 * 4 images, and _nb_cols = 2, then the images will be placed as 2 x 2 matrix.
 * But for instance, if we put _nb_cols = 3, in that case, would be 3 elements in the
 * first row, and one element in the second one.
 *
 *  The amount of widgets is fixed, and it does not dependend on the received
 * data. Nonetheless, this number must be fixed to a value that ensures that
 * all the image modes produce the same amount or less images than the number
 * of widgets set. Now, this value is set to 4.
*/
class ImageGridWidget : public QWidget
{
public:
    /**
     * @brief Constructor.
     *
     * @arg parent: Parent widget. By default, it is zero.
    */
    ImageGridWidget(QWidget* parent = 0);
    ~ImageGridWidget();

    /**
     * @brief updateGridImages: Update the shown images by each ImageWidget on
     *      the grid, and show the corresponding image labels. Internally,
     *      we have set 12 widgets with 4 columns maximum. The amount of columns
     *      used can be set with colLimit.
     *
     * @arg new_images: Vector of OpenCV image type cv::Mat. It contains the
     *      new images to be shown.
     *
     * @arg colLimit: Amount of columns to use. This value can be 1, 2, 3, 4. For
     *      instance, if the amount of widgets is 12, and we have 4 images,
     *      we can arrange them in several shapes. We can do a single column
     *      of 4 elements (colLimit == 1), two columns of 2 elements
     *      (colLimit == 2), 3 elements in the first row, and one element
     *      in the second row (colLimit == 3), or 4 elements in the first row only.
    */
    void updateGridImages(std::vector<cv::Mat> &new_images, int colLimit);

    /**
     * @brief clearBuffers: Remove all the information stored in all the
     *      ImageWidget objects used in the grid. After calling this function,
     *      the ImageWidget objects will be blank. It also hides the image labels.
     */
    void clearBuffers();

    /**
     * @brief setWidgetState: Show or hide the widgets (images and labels).
     *      This function will call the show or hide function of the
     *      corresponding ImageWidget and its associated QLabel, identified by the id field.
     *
     * @arg id: Integer that corresponds to the position of the ImageWidget in the
     *      internal vector of objects of this type. The position is the same as
     *      the one given by the labels passed on contruction.
     *
     * @arg enabled: Boolean that tells if we want to show (true) or to hide
     *      the requested widgets.
    */
    void setWidgetState(int id, bool enabled);

    /**
     * @brief setImageLabels: Update the labels that will be used to identify
     *      the showed images.
     *
     * @arg newLabels: Vector of string with the new labels to be used.
    */
    void setImageLabels(std::vector<std::string> &newLabels) { _imageLabels = newLabels; }

    /**
     * @brief getImageWidth: Any received image is resized to have a fixed
     *      width. This function returns this width value. This parameter is
     *      the same for all the widgets in the grid.
     *
     * @returns: Integer with the image width
     */
    int getImageWidth() const;

    // \brief changeImageWidth: Update the fixed width value for all the shown images
    void changeImageWidth(int new_width);

private:
    /**
     * @brief initializeWidget: Initialize the widget structure. It will create
     *  the corresponding labels objects, the ImageWidgets, and it will set their
     *  initial state.
    */
    void initializeWidget();

    unsigned int _nb_widgets;
    int _nb_cols;
    std::vector<ImageWidget*> _images;
    std::vector<QLabel*> _labelWidgets;
    /**
     * Vector of strings. It contains the labels of the images
     * that will be shown. The order of these labels must be the same as the
     * order of the received vector when updating the shown images.
     */
    std::vector<std::string> _imageLabels;

    QGridLayout *_layout;
};
#endif // __IMAGE_GRID_WIDGET_HPP__