#ifndef __REAL_TIME_PARAMETERS_HPP__
#define __REAL_TIME_PARAMETERS_HPP__

// OpenCV includes

// STD includes
#include <string>

// Qt Includes
#include <QCheckBox>
#include <QLabel>
#include <QWidget>

// Pylon includes

// Custom includes
#include <CameraTypes.hpp>

/**
 * @brief ParametersSet: This struct serves to avoid to have a long parameters
 * list when we update the widget labels.
 *
 * @arg minInt: Minimum intensity
 * @arg maxInt: Maximum intensity
 * @arg aolp: Angle of linear polarization
 * @arg minDoLP: Minimum degree of linear polarization
 * @arg meanDoLP: Mean degree of linear polarization
 * @arg maxDoLP: Maximum degree of linear polarization
*/
typedef struct
{
    double minInt;
    double maxInt;
    double aolp;
    double minDoLP;
    double meanDoLP;
    double maxDoLP;
} ParametersSet;

/**
 * @brief RealTimeParameters: Widget to show the real time parameters of the
 * light. The parameters shown are: minimum and maximum intensity measured in
 * the image, AoLP, minimum, mean, and maximum degree of linear polarization.
 *
 *  The values shown are provided from external functions. Here, only the given
 * values are shown. A checkbox is provided to stop the labels updates.
*/
class RealTimeParameters : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor.
     *
     * @arg camImgFormat: Struct with the camera bit-depth information.
     * @arg parent: Parent widget
    */
    RealTimeParameters(std::shared_ptr<bitDepthStruct> camImgFormat, QWidget* parent = 0);
    ~RealTimeParameters();

    /**
     * @brief updateParamLabels: Update the labels values. All the parameters
     * of the struct params are used for it.
     *
     * @arg params: New parameters to be shown in the labels.
    */
    void updateParamLabels(ParametersSet params);

    /**
     * @brief isEnabled: Check if the enabling checkbox is enabled or not.
     *
     * @returns boolean. If true, the widget is enabled.
    */
    bool isEnabled() {return (_paramsEnableCheckbox->checkState() == Qt::Checked);}

protected slots:
    /**
     * @brief onChangeParamsCheckbox: Slot executed each time the checkbox state
     * changes.
     *
     * @arg state: New state of the checkbox.
    */
    void onChangeParamsCheckbox(int state);

private:
    /**
     * @brief initializeWidget: Initialize the widget structure.
    */
    void initializeWidget();

    /**
     * @brief createStringWithNumber: Create an string that is a combination of
     * a number and a string.
     *
     * @arg txt: Text to be place at the beginning of the string. No spaces are
     *  added between the number and the string, so this string must include
     *  a trailing space.
     * @arg number: Decimal number to be placed after the string.
     * @arg precision: Amount of decimals to include in the number.
     * @returns String with the join of the given text and the given number.
    */
    std::string createStringWithNumber(std::string txt, double number, int precision);

    // Internal variables
    QLabel* _aopLabel;
    QLabel* _maxValLabel;
    QLabel* _minValLabel;
    QLabel* _minDoLPLabel;
    QLabel* _meanDoLPLabel;
    QLabel* _maxDoLPLabel;
    QCheckBox* _paramsEnableCheckbox;

    std::shared_ptr<bitDepthStruct> _camImgFormat;
};

#endif // __REAL_TIME_PARAMETERS_HPP__