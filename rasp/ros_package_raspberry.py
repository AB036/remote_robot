#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic

import rospy
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import Motor
import ros_package
import Robot
import time

ros_node = ros_package.RosNodeRaspberry()
camera = PiCamera()
print("After picam")
camera.resolution = (640, 480)
camera.framerate = 5
rawCapture = PiRGBArray(camera, size=(640, 480))
print("After rawcapture")
generator = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
rawCapture.truncate(0)

left_motor = Motor.TrueMotor(17,27,22)
right_motor = Motor.TrueMotor(18,23,24)
robot = Robot.Robot(left_motor, right_motor, ros_node)

while not rospy.is_shutdown():
    print("In while")
    command = ros_node.command
    robot.process_command(command)
    frame = next(generator)
    image = frame.array
    cv2.putText(image, rospy.get_caller_id(), (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(image, command, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    ros_node.publish_image(image)
    rawCapture.truncate(0)
    time.sleep(0.2)


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
