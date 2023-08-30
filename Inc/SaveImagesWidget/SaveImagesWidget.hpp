#ifndef __SAVE_IMAGES_WIDGET_HPP__
#define __SAVE_IMAGES_WIDGET_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes
#include <string>
#include <vector>

// Qt Includes
#include <QLabel>
#include <QWidget>

// Custom includes

/**
 * @brief SaveImagesWidget class
 *  This class is in charge of showing the buttons and the labels in the interface
 * that helps to save an image in disk. This widget has two buttons, one to save
 * the current image, and another one to change the destination directory. The
 * current destination directory is shown as a label, below the buttons.
 *
 *  When the save button is pressed, it tries to create the destination path
 * directory, and then it request to the upper class to capture an image. Once
 * the image is received, the onIMageReceived slot must be called, and the images
 * must be provided. When this happens, a dialog is shown so the user can enter
 * a filename prefix. To this prefix, the date and hour of the snap will written
 * in the file name too, and the type of image too.
 *
 *  For example, if the prefix is polarized_images, and it is 14:14:14 of the day
 * 01/01/2021, and the polarization angle is 0, the final filename would be
 *          polarized_images_01012021_141414_color_0.png
 *
 *  NOTE: The name of the prefix is set by the machine time. So if the machine
 * time is incorrect, the date printed in the dates are incorrect too.
*/
class SaveImagesWidget : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     *
     * @arg parent: Parent widget. By default it is zero.
    */
    SaveImagesWidget(QWidget* parent = 0);
    ~SaveImagesWidget();

    /**
     * @brief onImagesReceived: Slot that must be called whenever the images
     *  are ready to be saved into disk. This slot must be called after
     *  receiving the signal requestCaptureImages, and not before.
     *
     * @arg images: Captured images that want to be saved into disk. This vector
     *  must be in concordance with the labels passed on construction. For instance,
     *  if the polarization image of 0 degrees is in the position
     *  1 in the images vector, the corresponding label must be in the same
     *  position in the labels vector.
    */
    void onImagesReceived(std::vector<cv::Mat> &images);

    /**
     * @brief setImageLabels: Update the labels that will be used to identify
     *      the stored images.
     *
     * @arg newLabels: Vector of string with the new labels to be used. These
     *      labels will be placed at the end of the filename.
    */
    void setImageLabels(std::vector<std::string> &newLabels) { _labels = newLabels; }

signals:
    /**
     * @brief requestCaptureImages: Signal emitted whenever the user clicks the
     * Store Image button. When this signal is emitted, it is ensured that the
     * destination directory exists.
    */
    void requestCaptureImages();

protected slots:
    /**
     * @brief onStoreImage: Slot called whenever the Store Image button is pressed.
     *  This slot will verify if the destination directory exists. If it does not
     * exists, it will create it, and if it succeeded, then signal
     * requestCaptureImages is emitted.
    */
    void onStoreImage();

    /**
     * @brief onChangeDirectory: Slot executed whenever the user clicks the
     *  button Change Directory. It will show a explorer dialog, and the user
     *  can choose the desired directory where to store the images when the
     *  Store images button is pressed. The path of the selected directory will
     *  be stored internally in this class.
    */
    void onChangeDirectory();

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

    std::string _storePath;
    QLabel* _storeDirectoryPath;
    unsigned int _pathLength;
     /**
     * std::vector with the strings that corresponds to the
     * polarization angles. These labels must be in the same order than the
     * images that we will store when the onImagesReceived slot is called.
     * For instance, when we work with the polarization images, if the
     * polarization image of 0 degrees is in the position
     * 1 in the images vector, the corresponding label must be in the same
     * position in the labels vector.
     */
    std::vector<std::string> _labels;
};

#endif // __SAVE_IMAGES_WIDGET_HPP__