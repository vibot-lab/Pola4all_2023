#ifndef __IMAGES_COVERTER_HPP__
#define __IMAGES_COVERTER_HPP__

// OpenCV includes
#include "opencv2/core/core.hpp"

// STD includes

// Qt Includes
#include <QImage>
#include <QPixmap>

// Custom includes

// Taken from https://asmaloney.com/2013/11/code/converting-between-cvmat-and-qimage-or-qpixmap/
/**
   Functions to convert between OpenCV's cv::Mat and Qt's QImage and QPixmap.

   Andy Maloney
   https://asmaloney.com/2013/11/code/converting-between-cvmat-and-qimage-or-qpixmap
**/

/*
   Endianness
   ---

   Although not totally clear from the docs, some of QImage's formats we use here are
   endian-dependent. For example:

      Little Endian
         QImage::Format_ARGB32 the bytes are ordered:    B G R A
         QImage::Format_RGB32 the bytes are ordered:     B G R (255)
         QImage::Format_RGB888 the bytes are ordered:    R G B

      Big Endian
         QImage::Format_ARGB32 the bytes are ordered:    A R G B
         QImage::Format_RGB32 the bytes are ordered:     (255) R G B
         QImage::Format_RGB888 the bytes are ordered:    R G B

   Notice that Format_RGB888 is the same regardless of endianness. Since OpenCV
   expects (B G R) we need to swap the channels for this format.

   This is why some conversions here swap red and blue and others do not.

   This code assumes little endian. It would be possible to add conversions for
   big endian machines though. If you are using such a machine, please feel free
   to submit a pull request on the GitHub page.
*/
#if Q_BYTE_ORDER == Q_BIG_ENDIAN
#error "Some of QImage\'s formats are endian-dependant. This file assumes little endian. See comment at top of header."
#endif

class ImagesConverter
{
public:
    ImagesConverter() {}
    ~ImagesConverter() = default;

   /**
    * @brief cvMatToQImage: Convert a given OpenCV cv::Mat image into a Qt QImage
    *
    * @arg: inMat: Input image. Only the most common OpenCV image formats has been
    *    handled: CV_8UC4, CV_8UC3, CV_16UC3, CV_16UC1, CV_8UC1
    *
    * @returns Corresponding converted QImage
   */
    static QImage cvMatToQImage(const cv::Mat &inMat);

   /**
    * @brief cvMatToQPixmap: Convert a given OpenCV cv::Mat image into a Qt QPixMap
    *
    * @arg: inMat: Input image. Only the most common OpenCV image formats has been
    *    handled: CV_8UC4, CV_8UC3, CV_16UC3, CV_16UC1, CV_8UC1
    *
    * @returns Corresponding converted QPixMap
   */
    static QPixmap cvMatToQPixmap(const cv::Mat &inMat);
};

#endif // __IMAGES_COVERTER_HPP__
