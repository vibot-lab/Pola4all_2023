#! /usr/bin/env python3
import rospy
import os
from mainwindow import MainWindow
from PyQt5.QtWidgets import QApplication
import sys

dirname = os.path.dirname(os.path.realpath(__file__))

def main():
    rospy.init_node('camera_control', anonymous=True)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    return 0

if __name__ == '__main__':
    main()
