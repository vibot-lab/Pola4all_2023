import cv2
import numpy as np

from PyQt5.QtGui import \
    QImage, \
    QPixmap, \
    QPainter, \
    qRgb

from PyQt5.QtWidgets import \
    QWidget

'''
Show a Numpy / OpenCV image.

This class uses Qt functions to show a given image as a Widget. This way,
we can attach that image to our program directly.

The shown images have a fixed width of 480 pixels (random, it can be changed),
We can show gray level images, or color images.
The shown image can be updated at any time.

IMPORTANT: The input image, if it is color, it is considered to be in BGR, and
not in RGB. If you want to provide an RGB image instead, the updateImage
method must be modified.
'''
class ImageWidget(QWidget):
    def __init__(self, parent=0):
        '''
        Constructor
        '''
        super(QWidget, self).__init__(parent)

        self.desired_width = 480
        self.showed_image = None
        self.clearBuffers()
        self.scaling_factor = 1

        self.grayscale_colortable = np.array([qRgb(i, i, i) for i in range(256)])

    def updateImage(self, new_image):
        '''
        Change the shown image. The image provided must not be empty.
        If it is a gray level image, we show it as an indexed image, with a
        gray-level palette (There is not another way to do it in Qt), or if
        it is a color image, we convert it into RGB and we show it as it is.

        In this function, the provided image is resized to have a fixed width
        set on construction, and to keep the aspect ratio.
        '''
        if new_image.size:
            # We compute the required scaling factor, for the desired width we
            # want to set the in image.
            self.scaling_factor = float(self.desired_width) / new_image.shape[1]
            dim = (self.desired_width, int(new_image.shape[0] * self.scaling_factor))
            # We resize the image to have the desired width, and keep the
            # aspect ratio
            scaled_image = cv2.resize(new_image, dim, interpolation=cv2.INTER_LINEAR)

            # We set the Widget size, so it does not take an unlimited space in
            # the MainWindow.
            self.setFixedSize(
                dim[0],
                dim[1])

            # import ipdb; ipdb.set_trace()

            if len(scaled_image.shape) == 2:
                # We create a QImage as an indexed image to show the grayscale
                # values. Because it is in indexed format, we set its color table
                # too to be grayscale.
                self.showed_image = QImage(
                    scaled_image,
                    dim[0],
                    dim[1],
                    dim[0],
                    QImage.Format_Indexed8)
                self.showed_image.setColorTable(self.grayscale_colortable)
            else:
                # If it is a color image, we convert from BGR (format in OpenCV),
                # to RGB. If a RGB image want to be provided, this this line must
                # be erased.
                scaled_image = cv2.cvtColor(scaled_image, cv2.COLOR_BGR2RGB)

                # We convert the input image into a QImage, to be shown by Qt
                self.showed_image = QImage(
                    scaled_image,
                    dim[0],
                    dim[1],
                    3*dim[0],
                    QImage.Format_RGB888)

            # We schedule a repaint, in order to update what we show in the widget
            self.repaint()
        else:
            print("Not updating image. The provided image is empty")


    def clearBuffers(self):
        '''
        This method erases the shown image. After calling this method, the widget
        will be blank
        '''
        self.showed_image = QImage()
        self.repaint()

    def mousePressEvent(self, event):
        '''
        Overloaded function from QtWidget. Whenver we click on the image,
        this function will be called. In this function we can retrieve the coordinates
        of the pixel touched with the mouse.
        '''
        # NOTE: X corresponds to the columns, and Y corresponds to the row of the ORIGINAL image
        row = int(event.pos().y() / self.scaling_factor)
        column = int(event.pos().x() / self.scaling_factor)
        print("You have pressed the pixel: (row, column) = ({}, {})".format(row, column))

    def paintEvent(self, event):
        '''
        Overloaded function from Qt. This function is the one that changes what
        we draw in the QWidget. We can draw lines, circles, rectangles, or other
        shapes, if we add the required lines into this function.

        This implementation only draws the image set when calling updateImage.
        '''
        if not self.showed_image is None:
            painter = QPainter(self)
            painter.drawPixmap(0,
                0,
                self.showed_image.width(),
                self.showed_image.height(),
                QPixmap(self.showed_image))