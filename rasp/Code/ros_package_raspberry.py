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

# Ros Node initialization
ros_node = ros_package.RosNodeRaspberry()

# Camera initialization
camera = PiCamera(sensor_mode = 6)
camera.resolution = (640, 480)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=(640, 480))
generator = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
rawCapture.truncate(0)

# Motors and robot initialization. For debug, virtual motors are used
debug = False
if debug:
    left_motor = Motor.VirtualMotor("left")
    right_motor = Motor.VirtualMotor("right")
else:
    left_motor = Motor.TrueMotor(17)
    right_motor = Motor.TrueMotor(27)
robot = Robot.Robot(left_motor, right_motor, ros_node)

while not rospy.is_shutdown():
    # Get command from ros Node and process it
    command = ros_node.command
    robot.process_command(command)

    # Get image from Camera and send it to ros Node
    frame = next(generator)
    image = frame.array
    image = cv2.flip(image, -1)
    cv2.putText(image, rospy.get_caller_id(), (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(image, command, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    ros_node.publish_image(image)
    rawCapture.truncate(0)

    # Sleep a moment to avoid 100% CPU usage
    time.sleep(0.05)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
