import rospy
import time
import cv_bridge
from threading import Thread
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class RosNodeRaspberry(Thread):
    def __init__(self):
        Thread.__init__(self)
        print("Starting ROS node")
        rospy.init_node('virtualbot', anonymous=True)
        rospy.Subscriber('commands', String, self.__callback)
        self.__image_publisher = rospy.Publisher("video_frame", CompressedImage, queue_size=1)
        self.__bridge = cv_bridge.CvBridge()
        self.__command = "stop"
        self.__last_time = time.time()


    def publish_image(self, image):
        print("Send image")
        self.__image_publisher.publish(self.__bridge.cv2_to_compressed_imgmsg(image))

    def __callback(self, data):
        self.__last_time = time.time()
        rospy.loginfo(rospy.get_caller_id() + ' received ' + data.data)
        self.__command = data.data

    def __get_command(self):
        return self.__command

    command = property(__get_command)
