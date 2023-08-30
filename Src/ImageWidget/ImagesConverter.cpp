// OpenCV includes
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"

// STD includes

// Qt Includes
#include <QDebug>

// Custom includes
#include "ImageWidget/ImagesConverter.hpp"

// NOTE: This does not cover all cases - it should be easy to add new ones as required.
QImage ImagesConverter::cvMatToQImage(const cv::Mat &inMat)
{
   switch (inMat.type())
   {
   // 8-bit, 4 channel
   case CV_8UC4:
   {
      QImage image(inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_ARGB32);

      return image;
   }

   // 8-bit, 3 channel
   case CV_8UC3:
   {
      QImage image(inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_RGB888);

      return image;
   }

   // 16-bit, 3 channel
   case CV_16UC3:
   {
      cv::Mat output8BitsImage;
      // We convert the data type from 16bits to 8 bits, and before
      // convertion, we multiply by the factor 255 / 4095, in order to keep
      // the relation ships and do not saturate the colors
      inMat.convertTo(output8BitsImage, CV_8UC3, 255.0 / 4095.0);

      QImage image(output8BitsImage.data,
                  output8BitsImage.cols, output8BitsImage.rows,
                  static_cast<int>(output8BitsImage.step),
                  QImage::Format_RGB888);

      return image;
   }

   // 16-bit, 1 channel
   case CV_16UC1:
   {
      cv::Mat output8BitsImage;
      // We convert the data type from 16bits to 8 bits, and before
      // convertion, we multiply by the factor 255 / 4095, in order to keep
      // the relation ships and do not saturate the colors
      inMat.convertTo(output8BitsImage, CV_8UC1, 255.0 / 4095.0);

      QImage image(output8BitsImage.data,
                  output8BitsImage.cols, output8BitsImage.rows,
                  static_cast<int>(output8BitsImage.step),
                  QImage::Format_Grayscale8);

      return image;
   }

   // 8-bit, 1 channel
   case CV_8UC1:
   {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
      QImage image(inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_Grayscale8);
#else
      static QVector<QRgb> sColorTable;

      // only create our color table the first time
      if (sColorTable.isEmpty())
      {
         sColorTable.resize(256);

         for (int i = 0; i < 256; ++i)
         {
            sColorTable[i] = qRgb(i, i, i);
         }
      }

      QImage image(inMat.data,
                     inMat.cols, inMat.rows,
                     static_cast<int>(inMat.step),
                     QImage::Format_Indexed8);

      image.setColorTable(sColorTable);
#endif

      return image;
   }

   default:
      qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
      break;
   }

   return QImage();
}

QPixmap ImagesConverter::cvMatToQPixmap(const cv::Mat &inMat)
{
   return QPixmap::fromImage(cvMatToQImage(inMat));
}
