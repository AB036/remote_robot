import Motor
import Robot
import time

left_motor = Motor.TrueMotor(17,27,22)
right_motor = Motor.TrueMotor(18,23,24)
robot = Robot.Robot(left_motor, right_motor)

command_list = ["up", "down", "left", "right", "stop"]

for command in command_list:
    robot.process_command(command)
    time.sleep(2)