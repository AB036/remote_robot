#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2



rosnode = RosNodeRaspberry()
camera = PiCamera()
print("After picam")
camera.resolution = (640, 480)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(640, 480))
print("After rawcapture")
generator = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
rawCapture.truncate(0)

while not rospy.is_shutdown():
    print("In while")
    command = rosnode.command
    frame = next(generator)
    image = frame.array
    cv2.putText(image, rospy.get_caller_id(), (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(image, command, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    rosnode.publish_image(image)
    rawCapture.truncate(0)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
