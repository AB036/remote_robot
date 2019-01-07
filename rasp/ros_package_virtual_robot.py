#!/usr/bin/env python

import ros_package
import numpy as np
import rospy
import time
import cv2
import Motor
import Robot

rosnode = ros_package.RosNodeRaspberry()
left_motor = Motor.VirtualMotor("left")
right_motor = Motor.VirtualMotor("right")
robot = Robot.Robot(left_motor, right_motor)

for i in range(10):
    command = rosnode.command
    robot.process_command(command)
    image = blank_image = np.zeros((480, 640, 3), np.uint8)
    cv2.putText(image, command, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    rosnode.publish_image(image)
    time.sleep(0.1)
