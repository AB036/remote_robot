#!/usr/bin/env python

import ros_package
import numpy as np
import rospy
import time
import cv2
import Motor
import Robot

ros_node = ros_package.RosNodeRaspberry()
left_motor = Motor.VirtualMotor("left")
left_motor = Motor.TrueMotor(17,27,22)

right_motor = Motor.VirtualMotor("right")
right_motor = Motor.TrueMotor(18,23,24)
robot = Robot.Robot(left_motor, right_motor, ros_node)
command_list = ["up", "down", "left", "right", "stop"]

while True:
    command = ros_node.command
    robot.process_command(command)
    image = blank_image = np.zeros((480, 640, 3), np.uint8)
    cv2.putText(image, command, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    ros_node.publish_image(image)
    time.sleep(0.1)
