from PyQt5.QtWidgets import \
    QCheckBox, \
    QFileDialog, \
    QGroupBox, \
    QGridLayout, \
    QHBoxLayout, \
    QLabel, \
    QMainWindow, \
    QPushButton, \
    QScrollArea, \
    QSpinBox, \
    QVBoxLayout, \
    QWidget

from PyQt5.QtCore import \
    QTimer, \
    Qt

from PyQt5.QtGui import \
    QFont

from slider_with_label import SliderWithLabel
from image_widget import ImageWidget
from ros_comm import RosComm
from rosbag_handler import RosBagHandler
from cv_bridge import CvBridge

import cv2
import numpy as np

'''
MainWidow class: This widget is a GUI that allows to modulate the camera parameters
easily, and to show the images stored into a ROS Bag file.
'''
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        '''
        Constructor
        '''
        super(MainWindow, self).__init__(parent)
        self.right_layout = QVBoxLayout()
        self.left_layout = QVBoxLayout()
        self.main_layout = QHBoxLayout()

        # Internal widget variables
        self.exposure_spinbox = None
        self.frame_rate_spinbox = None
        self.gain_slider = None
        self.auto_exposure_cb = None
        self.auto_gain_cb = None
        self.auto_white_balance_cb = None
        self.red_gain_slider = None
        self.green_gain_slider = None
        self.blue_gain_slider = None
        self.max_label = None
        self.max_label_prefix = "Max. value: {}"
        self.min_label = None
        self.min_label_prefix = "Min. value: {}"
        self.is_first_valid_img = False

        self.raw_img_widgets = []
        self.ros_communication = RosComm()
        self.rosbag_handler = RosBagHandler()
        self.bridge_obj = CvBridge()

        self.initialize_widget()
        self.setWindowTitle("Camera controller software")

        # Update image timer
        self.image_update_timer = QTimer(self)
        self.image_update_timer.setInterval(20)
        self.image_update_timer.timeout.connect(self.on_image_update)
        self.image_update_timer.stop()

        # ROS bag reading timer
        self.rosbag_update_timer = QTimer(self)
        self.rosbag_update_timer.setInterval(5)
        self.rosbag_update_timer.timeout.connect(self.on_read_new_msg)
        self.rosbag_update_timer.stop()

        self.image_update_timer.start()

    def initialize_widget(self):
        '''
        Initialize all the widgets that forms the MainWindow.
        The structure of the MainWindow is splited into two regions: Left and Right.
        The left section shows the images, and the right section contains all the
        control widgets (sliders, spinboxes, etc).
        '''
        ## Here you start initializing your widgets
        self.add_image_widget()
        self.add_open_bag_button()
        self.add_size_spinbox()
        self.add_image_params_labels()
        self.add_exposure_time_spinbox()
        self.add_gain_slider()
        self.add_white_balance_sliders()
        self.add_framerate_spinbox()

        # Add the widgets that we want to add to the MainWindow must be added
        # before this line
        self.set_main_window_layouts()

    def set_main_window_layouts(self):
        '''
        Set the MainWindow Layout. We change the default MainWindow widget
        to be in two parts: Left and Right. So we can, for instance, show
        images in the left, and have control buttons to the right.

        For that, two layouts exists in the program: self.left_layout and
        self.right_layout.
        '''
        # We create scroll area to allows to slide the window, when the screen
        # size is not enough
        left_scroll = QScrollArea()
        right_scroll = QScrollArea()

        left_scroll.setWidgetResizable(True)
        right_scroll.setWidgetResizable(True)

        # The MainWindow is splitted into two areas, left and right, each of them
        # with the corresponding layouts
        left_widget = QWidget()
        right_widget = QWidget()
        main_widget = QWidget()

        left_widget.setLayout(self.left_layout)
        left_scroll.setWidget(left_widget)
        self.right_layout.addStretch(1)
        right_widget.setLayout(self.right_layout)
        right_scroll.setWidget(right_widget)

        self.main_layout.addWidget(left_scroll)
        self.main_layout.addWidget(right_scroll)
        main_widget.setLayout(self.main_layout)

        self.setCentralWidget(main_widget)

    def add_open_bag_button(self):
        '''
        This function adds two buttons: Open a ROS bag file with the software,
        and Stop reading, which will abruptly stop the reading procedure.

        This functionality is done by a QTimer. When a ROS bag want to be read,
        the QTimer to update from the ROS communication is stopped, and a QTimer
        is started to read one by one the images stored in the ROS bag file.

        This last timer can be stopped at any moment by the button Stop reading
        bag, which will restore the QTimer for the ROS communication service.
        '''
        self.start_button = QPushButton("Open RosBag file", self)
        self.stop_button = QPushButton("Stop reading bag", self)

        self.right_layout.addWidget(self.start_button)
        self.right_layout.addWidget(self.stop_button)

        self.start_button.clicked.connect(self.on_read_rosbag)
        self.stop_button.clicked.connect(self.on_stop_rosbag_reading)

        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def add_size_spinbox(self):
        '''
        Add a spinbox to change the width of the shown image.
        '''
        tempWidget = QWidget(self)

        layout = QHBoxLayout(tempWidget)
        label = QLabel("Image width:", tempWidget)
        spinbox = QSpinBox(tempWidget)
        spinbox.setMinimum(10)
        spinbox.setMaximum(3000)
        spinbox.setValue(self.raw_img_widgets[0].desired_width)
        layout.addWidget(label)
        layout.addWidget(spinbox)
        tempWidget.setLayout(layout)
        self.right_layout.addWidget(tempWidget)

        spinbox.valueChanged.connect(self.on_image_width_changed)

    def add_image_params_labels(self):
        '''
        Add two labels that will hold the minimum and maximum intensity values
        in the image. These labels will be updated each time a new image arrives.
        '''
        group = QGroupBox("Image parameters", self)
        layout = QVBoxLayout(group)

        custom_font = QFont("Arial", 14, QFont.Bold, True)
        self.max_label = QLabel(self.max_label_prefix.format(0), group)
        self.min_label = QLabel(self.min_label_prefix.format(0), group)
        self.min_label.setFont(custom_font)
        self.max_label.setFont(custom_font)

        layout.addWidget(self.min_label)
        layout.addWidget(self.max_label)
        group.setLayout(layout)
        self.right_layout.addWidget(group)

    def add_exposure_time_spinbox(self):
        '''
        Add a spinbox to change the exposure time of the camera.
        '''
        group = QGroupBox("Exposure control", self)
        layout = QVBoxLayout(group)

        # Spinbox with label widget
        tempWidget = QWidget(group)
        formLayout = QHBoxLayout(tempWidget)
        self.exposure_spinbox = QSpinBox(tempWidget)
        self.exposure_spinbox.setSuffix(" uS")
        self.exposure_spinbox.setMinimum(22)
        self.exposure_spinbox.setMaximum(1000000)
        self.exposure_spinbox.setSingleStep(100)
        label = QLabel("Exposure time: ", tempWidget)
        formLayout.addWidget(label)
        formLayout.addWidget(self.exposure_spinbox)
        tempWidget.setLayout(formLayout)

        self.auto_exposure_cb = QCheckBox("Auto exposure time", self)

        layout.addWidget(tempWidget)
        layout.addWidget(self.auto_exposure_cb)

        group.setLayout(layout)
        self.right_layout.addWidget(group)
        self.exposure_spinbox.valueChanged.connect(self.on_exposure_time_changed)
        self.auto_exposure_cb.stateChanged.connect(self.on_auto_exposure_changed)

    def add_gain_slider(self):
        '''
        Add a slider to change the gain in the camera.
        '''
        group = QGroupBox("Gain control", self)
        layout = QVBoxLayout(group)

        # Slider with label widget
        tempWidget = QWidget(group)
        tempLayout = QHBoxLayout(tempWidget)
        self.gain_slider = SliderWithLabel(0, 24, "dB", 2, tempWidget)
        label = QLabel("Gain: ", tempWidget)

        tempLayout.addWidget(label)
        tempLayout.addWidget(self.gain_slider)
        tempWidget.setLayout(tempLayout)

        self.auto_gain_cb = QCheckBox("Auto gain", self)

        layout.addWidget(tempWidget)
        layout.addWidget(self.auto_gain_cb)

        group.setLayout(layout)
        self.right_layout.addWidget(group)
        self.gain_slider.valueChanged.connect(self.on_gain_changed)
        self.auto_gain_cb.stateChanged.connect(self.on_auto_gain_changed)

    def add_image_widget(self):
        '''
        Add four image widgets that will show the demosaicked images at each
        orientation of the linear filters. The top left widget contains the image
        at 135 degrees, the top right shows the image at 0 degrees,
        the bottom left is the image at 90 degrees, and the bottom right corresponds
        to the image at 45 degrees.
        '''
        # We create a grid of 2x2 widgets to show the images
        temp = QWidget(self)
        layout = QGridLayout(temp)
        self.raw_img_widgets.append(ImageWidget(temp))
        self.raw_img_widgets.append(ImageWidget(temp))
        self.raw_img_widgets.append(ImageWidget(temp))
        self.raw_img_widgets.append(ImageWidget(temp))
        layout.addWidget(self.raw_img_widgets[0], 0, 0)
        layout.addWidget(self.raw_img_widgets[1], 0, 1)
        layout.addWidget(self.raw_img_widgets[2], 1, 0)
        layout.addWidget(self.raw_img_widgets[3], 1, 1)
        temp.setLayout(layout)
        self.left_layout.addWidget(temp)

    def add_white_balance_sliders(self):
        '''
        Add the corresponding controls for the white balance. It consists of
        three sliders that can have values between 1 and 15. This white balance
        is not done by software, but by hardware, which is useless for this case.
        We add them in order to ensure a valid acquisition set-up of the camera.
        '''
        group = QGroupBox("White balance", self)
        layout = QVBoxLayout(group)

        # Red gain slider
        red_gain_widget = QWidget(group)
        red_layout = QHBoxLayout(red_gain_widget)
        self.red_gain_slider = SliderWithLabel(1, 15, "", 1, red_gain_widget)
        red_label = QLabel("Red gain: ", red_gain_widget)

        red_layout.addWidget(red_label)
        red_layout.addWidget(self.red_gain_slider)
        red_gain_widget.setLayout(red_layout)
        ####

        # Green gain slider
        green_gain_widget = QWidget(group)
        green_layout = QHBoxLayout(green_gain_widget)
        self.green_gain_slider = SliderWithLabel(1, 15, "", 1, green_gain_widget)
        green_label = QLabel("Green gain: ", green_gain_widget)

        green_layout.addWidget(green_label)
        green_layout.addWidget(self.green_gain_slider)
        green_gain_widget.setLayout(green_layout)
        ####

        # Blue gain slider
        blue_gain_widget = QWidget(group)
        blue_layout = QHBoxLayout(blue_gain_widget)
        self.blue_gain_slider = SliderWithLabel(1, 15, "", 1, blue_gain_widget)
        blue_label = QLabel("Blue gain: ", blue_gain_widget)

        blue_layout.addWidget(blue_label)
        blue_layout.addWidget(self.blue_gain_slider)
        blue_gain_widget.setLayout(blue_layout)
        ####

        self.auto_white_balance_cb = QCheckBox("Auto white balance", self)

        layout.addWidget(red_gain_widget)
        layout.addWidget(green_gain_widget)
        layout.addWidget(blue_gain_widget)
        layout.addWidget(self.auto_white_balance_cb)

        group.setLayout(layout)
        self.right_layout.addWidget(group)
        self.red_gain_slider.valueChanged.connect(self.on_white_balance_changed)
        self.green_gain_slider.valueChanged.connect(self.on_white_balance_changed)
        self.blue_gain_slider.valueChanged.connect(self.on_white_balance_changed)
        self.auto_white_balance_cb.stateChanged.connect(self.on_auto_white_balance_changed)

    def add_framerate_spinbox(self):
        '''
        Add a spinbox to change the camera frame rate.
        '''
        tempWidget = QWidget(self)
        layout = QHBoxLayout(tempWidget)
        self.frame_rate_spinbox = QSpinBox(tempWidget)
        label = QLabel("Frame rate:", tempWidget)
        layout.addWidget(label)
        layout.addWidget(self.frame_rate_spinbox)
        tempWidget.setLayout(layout)
        self.right_layout.addWidget(tempWidget)

        self.frame_rate_spinbox.setMinimum(1)
        self.frame_rate_spinbox.setMaximum(100)
        self.frame_rate_spinbox.valueChanged.connect(self.on_frame_rate_changed)

    # Change the widget state from outside
    def set_global_state(self):
        '''
        Set the widget values to the values set in the camera. This function will
        query the camera state through a ROS service, and the received values will
        be used to set the MainWindow Widget's values.
        '''
        state = self.ros_communication.get_camera_parameters()
        self.exposure_spinbox.setValue(state[0])
        self.gain_slider.setValue(state[1])
        self.frame_rate_spinbox.setValue(state[2])
        self.red_gain_slider.setValue(state[3])
        self.green_gain_slider.setValue(state[4])
        self.blue_gain_slider.setValue(state[5])

        self.auto_exposure_cb.blockSignals(True)
        self.auto_gain_cb.blockSignals(True)
        self.auto_white_balance_cb.blockSignals(True)

        cb_state = Qt.Unchecked
        if state[6]:
            cb_state = Qt.Checked
        self.auto_gain_cb.setCheckState(cb_state)

        cb_state = Qt.Unchecked
        if state[7]:
            cb_state = Qt.Checked
        self.auto_exposure_cb.setCheckState(cb_state)

        cb_state = Qt.Unchecked
        if state[8]:
            cb_state = Qt.Checked
        self.auto_white_balance_cb.setCheckState(cb_state)

        self.gain_slider.setEnabled(not state[6])
        self.exposure_spinbox.setEnabled(not state[7])
        self.red_gain_slider.setEnabled(not state[8])
        self.green_gain_slider.setEnabled(not state[8])
        self.blue_gain_slider.setEnabled(not state[8])

        self.auto_exposure_cb.blockSignals(False)
        self.auto_gain_cb.blockSignals(False)
        self.auto_white_balance_cb.blockSignals(False)

    ## Internal widget callbacks
    def on_exposure_time_changed(self, value):
        '''
        Action taken each time we change the exposure time spinbox value
        '''
        self.ros_communication.change_exposure_time(value)

    def on_gain_changed(self, value):
        '''
        Action taken each time we change the gain slider value.
        '''
        self.ros_communication.change_gain(value)

    def on_image_update(self):
        '''
        QTimer timeout callback. This function is executed periodically, when the
        update image QTimer is running.

        It will read the last received image in the ROS communication module,
        it will apply a scaling factor to the intensity values, and convert it
        into uint8 datatype. Finally, it will separate the images by angle of
        polarization, and it will demosaick them. The final images will be passed
        to the Image Widgets to show them.
        '''
        img = self.ros_communication.get_raw_image()

        # We check if this is the first valid image received
        if (not self.is_first_valid_img) and (not img is None):
            self.set_global_state()
            self.is_first_valid_img = True

        if not img is None:
            self.min_label.setText(self.max_label_prefix.format(np.min(img)))
            self.max_label.setText(self.min_label_prefix.format(np.max(img)))

            if img.dtype == np.uint16:
                img = np.array((img * 255.0 / 4095.0), dtype=np.uint8)
            elif img.dtype == np.uint8:
                pass
            else:
                print("ERROR: Received image format not recognized: {}".format(img.dtype))
                assert(0)

            images = self.convert_raw_image(img)
            for i in range(len(self.raw_img_widgets)):
                self.raw_img_widgets[i].updateImage(images[i])

    def on_image_width_changed(self, value):
        '''
        Callback executed each time the image width spinbox value is changed.
        '''
        for widget in self.raw_img_widgets:
            widget.desired_width = value

    def on_frame_rate_changed(self, value):
        '''
        Callback executed each time the frame rate spinbox value is changed.
        '''
        self.ros_communication.update_frame_rate(value)

    def on_free_run_start(self):
        '''
        Slot executed each time we want to put the camera in continuous mode. It
        will call the corresponding service to start the mode, and then it will
        request the camera state through another service. Finally, it will set
        this state into the different GUI widgets.
        '''
        self.set_global_state()

    def on_free_run_stop(self):
        '''
        Slot executed each time we want to stop the continuous mode in the camera.
        After calling the corresponding service, all the image widgets buffers
        will be erased.
        '''
        for i in range(len(self.raw_img_widgets)):
            self.raw_img_widgets[i].clearBuffers()

    def on_read_rosbag(self):
        '''
        Slot executed each time the user wants to read ROS bag. This call back will
        open a Pop-up window, in which it will ask for the file it wants to open.
        Then, if the file is a valid ROS bag, it will start a timer that will update
        the images shown in the image widgets periodically, through a QTimer.

        During this operation, the QTimer that updates the images through the
        ROS communication is disabled. The ROS bag lecture can be cancelled at any
        time, through the boutton Stop reading.
        '''
        fname = QFileDialog.getOpenFileName(self,
            'Open ROS bag file',
            '',
            "ROS Bag file ( *.bag )")
        if fname[0]:
            # In this case, we read the images topic. If another topic want to be
            # read, this parameter must be changed.
            if self.rosbag_handler.load_ros_bag(fname[0], '/camera_image_topic'):
                # We stop the topic timer
                self.image_update_timer.stop()
                self.start_button.setEnabled(False)
                self.stop_button.setEnabled(True)
                self.rosbag_update_timer.start()
            else:
                print("ERROR: Cannot open the ROS bag file: {}".format(fname[0]))
        else:
            print("No path value entered.")

    def on_read_new_msg(self):
        '''
        Callback executed periodically, through a QTimer. At each iteration,
        it will take one message (image message) from the ROS bag, and it will
        show it in the different Image Widgets. Once it finishes the bag, it
        will clear the buffers of the image widgets.
        '''
        if self.rosbag_handler.is_bag_opened():
            msg = self.rosbag_handler.get_single_msg()

            if msg is None:
                print("End of ROS bag")
                self.on_stop_rosbag_reading()
                return

            img = self.bridge_obj.imgmsg_to_cv2(msg, desired_encoding="mono16")
            if img.dtype == np.uint16:
                img = np.array((img * 255.0 / 4095.0), dtype=np.uint8)
            elif img.dtype == np.uint8:
                pass
            else:
                print("ERROR: Received image format not recognized: {}".format(img.dtype))
                assert(0)

            images = self.convert_raw_image(img)
            for i in range(len(self.raw_img_widgets)):
                self.raw_img_widgets[i].updateImage(images[i])
        else:
            print("ERROR: ROS bag is not loaded yet!")
            self.on_stop_rosbag_reading()

    def on_stop_rosbag_reading(self):
        '''
        Slot executed each time we want to stop the ROS bag reading. It will clear
        the image widgets buffers, and it will restore the QTimer that reads the
        ros topics.
        '''
        self.rosbag_update_timer.stop()

        if self.rosbag_handler.is_bag_opened():
            self.rosbag_handler.close_bag()

        for i in range(len(self.raw_img_widgets)):
            self.raw_img_widgets[i].clearBuffers()

        # We restore the normal timers the topic timer
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.image_update_timer.start()

    def on_white_balance_changed(self, value):
        '''
        Slot executed each time any white balance gain is changed. At this moment,
        all the gains will be read from the sliders, and they will be sent to the
        camera.

        Args:
            value: Caller slider value. Not used.
        '''
        gains = [self.red_gain_slider.value(), self.green_gain_slider.value(), self.blue_gain_slider.value() ]
        self.ros_communication.set_white_balance_gains(gains)

    def convert_raw_image(self, img):
        '''
        Extract the different image channels depending the angle of polarization.

        Args:
            img: Raw mosaick image.

        Returns:
            List with four images: I_135, I_0, I_90 and I_45. All the images are
            demosaicked.
        '''
        I_135 = img[0::2, 0::2]
        I_0 = img[0::2, 1::2]
        I_90 = img[1::2, 0::2]
        I_45 = img[1::2, 1::2]

        rgb_135 = cv2.cvtColor(I_135, cv2.COLOR_BAYER_RG2RGB)
        rgb_0 =   cv2.cvtColor(I_0,   cv2.COLOR_BAYER_RG2RGB)
        rgb_90 =  cv2.cvtColor(I_90,  cv2.COLOR_BAYER_RG2RGB)
        rgb_45 =  cv2.cvtColor(I_45,  cv2.COLOR_BAYER_RG2RGB)

        return [rgb_135, rgb_0, rgb_90, rgb_45]

    def on_auto_exposure_changed(self, state):
        '''
        Slot executed each time the Auto exposure time checkbox changes its state.

        Args:
            state: New check box state
        '''
        self.ros_communication.set_auto_exposure_state(self.auto_exposure_cb.isChecked())
        self.exposure_spinbox.setEnabled(not self.auto_exposure_cb.isChecked())

        if not self.auto_exposure_cb.isChecked():
            self.set_global_state()

    def on_auto_gain_changed(self, state):
        '''
        Slot executed each time the Auto gain checkbox changes its state.

        Args:
            state: New check box state
        '''
        self.ros_communication.set_auto_gain_state(self.auto_gain_cb.isChecked())
        self.gain_slider.setEnabled(not self.auto_gain_cb.isChecked())

        if not self.auto_gain_cb.isChecked():
            self.set_global_state()

    def on_auto_white_balance_changed(self, state):
        '''
        Slot executed each time the Auto white balance checkbox changes its state.

        Args:
            state: New check box state
        '''
        self.ros_communication.set_auto_white_balance_state(self.auto_white_balance_cb.isChecked())
        self.red_gain_slider.setEnabled(not self.auto_white_balance_cb.isChecked())
        self.green_gain_slider.setEnabled(not self.auto_white_balance_cb.isChecked())
        self.blue_gain_slider.setEnabled(not self.auto_white_balance_cb.isChecked())

        if not self.auto_white_balance_cb.isChecked():
            self.set_global_state()