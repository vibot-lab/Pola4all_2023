import rosbag
import cv2
import numpy as np
import os
import rospy
from cv_bridge import CvBridge

'''
RosBagHandler class. It serves as an interface to open and read a ROS bag file.
It is oriented to read the messages of type images, but it can be used for any
other topic.

When we load a file, we read all the messages timestamps, and we store them. Then,
we can query a single message by providing the time at which it happened. This is
useful to retrieve a single message at the time from the entire ros bag. This way,
we do not have to read and store in RAM all the messages information, which accounts
for several Gb.
'''
class RosBagHandler(object):
    def __init__(self):
        '''
        Constructor.
        '''
        self.bagfile = None
        self.timestamps = []
        self.time_index = 1
        self.topic_of_interest = None

    def load_ros_bag(self, filename, topic_of_interest):
        '''
        Load a ROS bag file, and store a list with all the timestamps corresponding
        to a topic of interest. This way, we can get a single message at the time
        when reading the file.

        Args:
            filename: String with the full path where the ROS bag file is.
            topic_of_interest: Topic we want to read. It should start with the
            "/" character.
        '''
        ret_val = False
        self.timestamps.clear()
        self.bagfile = None
        self.time_index = 1
        self.topic_of_interest = topic_of_interest
        try:
            self.bagfile = rosbag.Bag(filename, 'r')
            ret_val = True
            for topic, msg, t in self.bagfile.read_messages(topics=[self.topic_of_interest]):
                self.timestamps.append(t)
        except FileNotFoundError:
            pass

        return ret_val

    def is_bag_opened(self):
        '''
        Check if the ROS bag file is opened or not.
        '''
        return (not self.bagfile is None)

    def close_bag(self):
        '''
        Close the ROS bag file, and reset the corresponding variables.
        '''
        if not self.bagfile is None:
            self.bagfile.close()
            self.bagfile = None

        self.timestamps.clear()
        self.time_index = 1
        self.topic_of_interest = None

    def get_single_msg(self):
        '''
        Get a single message from the bag. This message is the next in the queue
        of the file. The message pointer is set to one position in the timestamps
        list, created when opening the bag. The position of the pointer is updated
        automatically at each call to this function.

        Returns:
            ROS message at the current position pointer.
        '''
        ret_data = None
        if self.is_bag_opened():
            if self.time_index < len(self.timestamps):
                start_t = self.timestamps[self.time_index - 1]
                end_t = self.timestamps[self.time_index]
                ## The end time is included too, so we need to reduce the time by
                # 1 in the clock resolution to avoid double reading.
                end_t.nsecs = end_t.nsecs - 1
                self.time_index += 1

                for topic, msg, t in self.bagfile.read_messages(topics=[self.topic_of_interest], start_time=start_t, end_time=end_t):
                    ret_data = msg
        else:
            print("Cannot read the messages. The Bag file is not opened yet")

        return ret_data

if __name__ == '__main__':
    dirname = os.path.dirname(os.path.realpath(__file__))
    filepath = os.path.join(dirname, "..", "..", "..", "myBag_2022-01-17-09-16-23.bag")

    handler = RosBagHandler()
    handler.load_ros_bag(filepath, '/camera_image_topic')
    if not handler.is_bag_opened:
        print("Bag is not opened!")
    else:
        print("Bag sucessfully opened")

    bridge_obj = CvBridge()

    while True:
        msg = handler.get_single_msg()

        if msg is None:
            break

        img = bridge_obj.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        I_135 = img[0::2, 0::2]
        if img.dtype == np.uint16:
            I_135 = np.array(I_135 * 255.0 / 4095.0, dtype=np.uint8)
        elif img.dtype == np.uint8:
            pass
        else:
            print("ERROR: Received image format not recognized: {}".format(img.dtype))
            assert(0)

        I_135_color = cv2.cvtColor(I_135, cv2.COLOR_BAYER_RG2RGB)
        cv2.namedWindow("Hello", cv2.WINDOW_NORMAL)
        cv2.imshow("Hello", I_135_color)
        cv2.waitKey(1)

    cv2.destroyAllWindows()