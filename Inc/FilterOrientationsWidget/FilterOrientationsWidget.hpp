#ifndef __FILTER_ORIENTATIONS_WIDGET_HPP__
#define __FILTER_ORIENTATIONS_WIDGET_HPP__

// OpenCV includes

// STD includes
#include <map>
#include <vector>

// Qt Includes
#include <QComboBox>
#include <QDialog>
#include <QKeyEvent>
#include <QWidget>

// Custom includes

/**
 * @brief FilterOrientationsWidget: Widget to change the super-pixel
 *   configuration, which depends on the camera model we have. In a polarization
 *   camera, we have groups of 2 x 2 pixels, that each of them has a
 *   linear polarization filter, oriented differently. The orientations are
 *   in general the same, but their disposition might change. This widget
 *   provides a QDialog that allows to change this configuration, by changing
 *   the position of each filter.
 *
 *    It counts with four QComboBox, each of them with the four options of
 *   the filter orientations. It also has two buttons: Ok to commit the
 *   shown orientations, or cancel to restore the previous configuration.
 *   This widget will also check that two orientations are not repeated. If
 *   that's the case, it will show an error message, and it will restore the
 *   previous configuration.
*/
class FilterOrientationsWidget : public QDialog
{
    Q_OBJECT
public:
    /**
     * @brief FilterOrientationsWidget: Constructor. It will initialize the
     *   widget architecture, and set its title.
     *
     * @arg parent: Parent widget.
    */
    FilterOrientationsWidget(QWidget* parent = 0);
    ~FilterOrientationsWidget();

    /**
     * @brief getPixelsMap: Get the map between the filter orientations (in
     *   degrees) and the position at which they are in the 2x2 arrangement.
     *   The positions are given as numbers between 0 and 3. In the top row,
     *   from left to right, we find the indexes 0 and 1, and in the bottom row,
     *   the indexes 2 and 3.
     *
     * @returns: Map from int to int, in which the key is the orientation, and
     *   the value is index of the filter in the 2x2 matrix.
    */
    std::map<int, int> getPixelsMap() const
    {
        return std::map<int, int>({
            std::pair<int, int>(_optionsList[_currentIndexes[0]], 0),
            std::pair<int, int>(_optionsList[_currentIndexes[1]], 1),
            std::pair<int, int>(_optionsList[_currentIndexes[2]], 2),
            std::pair<int, int>(_optionsList[_currentIndexes[3]], 3)
        });
    }

    /**
     * @brief showDialog: Show this widget, and once the user click in the Ok
     *   button, it checks if there are repeated indexes. If there are, it will
     *   show an error message, and it will restore the previous configuration.
    */
    void showDialog();

signals:
    /**
     * @brief orientationsUpdated: Signal emitted each time the filter
     *   orientations configuration is changed.
     *
     * @arg newOrientationMap: Map of the orientation indexes. They key is
     *   the filter orientation (in degrees), and the value is its position
     *   in the array.
    */
    void orientationsUpdated(std::map<int, int> newOrientationMap);

protected:
    /**
     * @brief keyPressEvent: Overloaded function of a QWidget. It will be called
     *   each time a key is pressed when the dialog is focused. In this case,
     *   we reimplemented it to ignore the Esc key behavior.
     *
     * @arg e: Event parameters object.
    */
    void keyPressEvent(QKeyEvent *e) override;

    /**
     * @brief onAcceptedDialog: Slot executed each time the user presses the
     *   Ok button. It will just set of the accept flag of the QDialog.
    */
    void onAcceptedDialog();

    /**
     * @brief onRejectedDialog: Slot executed each time the user presses the
     *   Cancel button. It will restore the previous configuration, and it will
     *   set the reject flag of the QDialog.
    */
    void onRejectedDialog();

private:
    /**
     * @brief initializeDialog: Initialize the widget structure (Buttons,
     *   option lists, and task widget description).
    */
    void initializeDialog();

    /**
     * @brief getGridOfOptions: Create a widget with the four QComboBoxes of
     *   the orientation options.
     *
     * @returns: Pointer to a widget that contains a grid of 2x2 QComboBox widgets.
    */
    QWidget* getGridOfOptions();

    /**
     * @brief getComboBox: Get a QComboBox widget with the options already inserted.
     *
     * @arg initialIdx: Initial option index.
     * @arg outputSpinBoxPtr: QComboBox already initialized.
    */
    void getComboBox(
        int initialIdx,
        QComboBox* &outputSpinBoxPtr);

    /// \brief saveIndexes: Save the current configuration into an internal variable.
    void saveIndexes();

    /// \brief restoreLastIndexes: Restore the last saved configuration.
    void restoreLastIndexes();

    /**
     * @brief isThereDupplicates: Check if there are dupplicated options in the
     *   given list.
     *
     * @arg vec: Vector to evaluate.
     * @returns True if there are at least two equal elements in vec.
    */
    bool areThereDupplicates(std::vector<int> vec);

    const std::vector<int> _optionsList;
    std::vector<int> _currentIndexes;
    QComboBox* _filter0Idx;
    QComboBox* _filter1Idx;
    QComboBox* _filter2Idx;
    QComboBox* _filter3Idx;
};

#endif // __FILTER_ORIENTATIONS_WIDGET_HPP__